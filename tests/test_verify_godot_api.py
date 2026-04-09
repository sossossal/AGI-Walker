from tests.verify_godot_api import verify_api


def test_verify_godot_api_script() -> None:
    assert verify_api()
