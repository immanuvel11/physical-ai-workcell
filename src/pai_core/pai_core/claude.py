"""Thin wrapper around the Anthropic Python SDK for the planner and the perception labeller.

Credentials, in order: ANTHROPIC_API_KEY env var, the file ~/.config/pai/anthropic_api_key,
then whatever the SDK finds itself (e.g. an `ant auth login` profile).
Every call returns parsed JSON or None, so callers can always fall back to offline behaviour.
"""
import json
import os
import time

DEFAULT_MODEL = 'claude-opus-5-5'
KEY_FILE = os.path.expanduser('~/.config/pai/anthropic_api_key')
FALLBACK_BETA = 'server-side-fallback-2026-07-01'


class ClaudeClient:
    def __init__(self, logger, model=DEFAULT_MODEL):
        self.log = logger
        self.model = model
        self.client = None
        self.reason = ''
        self.last_latency = 0.0
        try:
            import anthropic
        except ImportError:
            self.reason = 'python package "anthropic" not installed (pip install --user anthropic)'
            return
        self._anthropic = anthropic
        key = os.environ.get('ANTHROPIC_API_KEY')
        if not key and os.path.exists(KEY_FILE):
            with open(KEY_FILE) as f:
                key = f.read().strip() or None
        try:
            client = anthropic.Anthropic(api_key=key, max_retries=2, timeout=60.0) if key \
                else anthropic.Anthropic(max_retries=2, timeout=60.0)
        except Exception as e:
            self.reason = f'cannot create Anthropic client ({e.__class__.__name__}: {e})'
            return
        # The SDK only complains about missing credentials when a request is sent: check now.
        if not (key or getattr(client, 'api_key', None) or getattr(client, 'auth_token', None)
                or getattr(client, 'credentials', None)):
            self.reason = f'no Anthropic API key (set ANTHROPIC_API_KEY or write it to {KEY_FILE})'
            return
        self.client = client
        import inspect
        self.has_fallbacks = 'fallbacks' in inspect.signature(client.beta.messages.create).parameters

    @property
    def available(self):
        return self.client is not None

    def json_call(self, system, content, schema, effort='medium', max_tokens=16000):
        """One Messages API call constrained to `schema`. Returns (dict | None, error_str)."""
        if not self.available:
            return None, self.reason
        a = self._anthropic
        kwargs = dict(
            model=self.model, max_tokens=max_tokens, system=system,
            messages=[{'role': 'user', 'content': content}],
            output_config={'effort': effort, 'format': {'type': 'json_schema', 'schema': schema}},
        )
        t0 = time.monotonic()
        try:
            if self.has_fallbacks:
                # Server-side refusal fallback: a declined request is re-run on a recommended model.
                resp = self.client.beta.messages.create(betas=[FALLBACK_BETA], fallbacks='default', **kwargs)
            else:  # older SDK without the `fallbacks` parameter
                resp = self.client.messages.create(**kwargs)
        except a.AuthenticationError:
            self.client, self.reason = None, 'Anthropic API key rejected'
            return None, self.reason
        except a.RateLimitError:
            return None, 'rate limited by the Anthropic API'
        except a.APIStatusError as e:
            return None, f'Anthropic API error {e.status_code}: {e.message}'
        except a.APIConnectionError:
            return None, 'cannot reach the Anthropic API (network)'
        except Exception as e:  # never let an SDK surprise take the robot pipeline down
            return None, f'Anthropic call failed: {e.__class__.__name__}: {e}'
        finally:
            self.last_latency = time.monotonic() - t0
        if resp.stop_reason == 'refusal':
            return None, 'request declined by the model'
        if resp.stop_reason == 'max_tokens':
            return None, 'response truncated (max_tokens)'
        text = next((b.text for b in resp.content if b.type == 'text'), None)
        try:
            return json.loads(text), ''
        except (TypeError, json.JSONDecodeError):
            return None, 'model returned invalid JSON'


def image_block(jpeg_bytes):
    import base64
    return {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg',
                                        'data': base64.standard_b64encode(jpeg_bytes).decode()}}
