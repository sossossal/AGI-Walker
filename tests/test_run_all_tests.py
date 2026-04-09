from tests.run_all_tests import run_all_tests


def test_run_all_tests_subset() -> None:
    assert run_all_tests(
        selected_ids=("parts_store", "dreamer_interface", "sim2real_score"),
    )
