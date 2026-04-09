from tests.verify_dreamer_interface import verify_dreamer_interface


def test_verify_dreamer_interface_script() -> None:
    assert verify_dreamer_interface(max_steps=6)
