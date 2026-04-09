from tests.verify_web_integration import verify_web_integration


def test_verify_web_integration_script() -> None:
    assert verify_web_integration()
