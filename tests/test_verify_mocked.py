import asyncio

from tests.verify_mocked import run_verification


def test_verify_mocked_script() -> None:
    assert asyncio.run(run_verification())
