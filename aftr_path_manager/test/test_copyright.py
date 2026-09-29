# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
#
# Licensed under the Apache License, Version 2.0

"""Copyright test."""

from ament_copyright.main import main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
@pytest.mark.skip(reason="Generated package resource files do not use headers.")
def test_copyright():
    """Test copyright."""
    rc = main(argv=[".", "test"])
    assert rc == 0
