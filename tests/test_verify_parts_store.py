from tests.verify_parts_store import verify_parts_store


def test_verify_parts_store_script() -> None:
    assert verify_parts_store()
