import os
import sys
import pytest

def test_import_pycrobot_kdl():
    """
    Verify that pycrobot.kdl can be imported without errors.
    Prints PYTHONPATH and sys.path if import fails.
    """
    try:
        from pycrobot import kdl
    except ImportError as e:
        # Print useful debug info
        print("=== DEBUG INFO ===")
        print(f"PYTHONPATH: {os.environ.get('PYTHONPATH')}")
        print("sys.path:")
        for p in sys.path:
            print(f"  {p}")
        print("==================")
        pytest.fail(f"Failed to import pycrobot.kdl: {e}")
    
    assert kdl is not None
