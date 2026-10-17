"""Blocking helpers for callbacks running in a MultiThreadedExecutor."""
import time


def wait_future(future, timeout=None, poll=0.01):
    """Wait for an rclpy future without spinning (another executor thread spins). Returns result or None."""
    end = None if timeout is None else time.monotonic() + timeout
    while not future.done():
        if end is not None and time.monotonic() > end:
            return None
        time.sleep(poll)
    return future.result()


def wait_until(predicate, timeout, poll=0.05):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(poll)
    return predicate()
