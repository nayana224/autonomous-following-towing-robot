# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Flake8 test."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Test flake8."""
    rc, errors = main_with_errors(argv=[".", "test"])
    assert rc == 0, "\n" + "\n".join(errors)
