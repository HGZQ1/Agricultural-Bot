"""Run the ROS 2 Python style checker."""

from ament_flake8.main import main_with_errors

import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check PEP 8 and import ordering."""
    return_code, errors = main_with_errors(argv=[])
    assert return_code == 0, (
        f'Found {len(errors)} code style errors:\n' + '\n'.join(errors))
