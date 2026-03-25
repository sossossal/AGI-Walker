import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import pytest
from web_panel.command_parser import CommandParser

def test_quadruped_parsing() -> None:
    parser = CommandParser()
    res = parser.parse("我要一个四足机器狗，高度 0.8 米，重量 25kg，大腿 0.3 米")
    
    # Assert type
    assert res["metadata"]["type"] == "quadruped"
    
    # Assert dimensions
    params = res["skills_params"]
    assert params["type"] == "quadruped"
    assert params["torso_height"] == 0.8
    assert params["torso_mass"] == 25.0
    assert params["thigh_length"] == 0.3

def test_biped_parsing() -> None:
    parser = CommandParser()
    res = parser.parse("帮我建一个双足机器人，高1.2m，重40千克")
    
    assert res["metadata"]["type"] == "biped"
    params = res["skills_params"]
    assert params["type"] == "biped"
    assert params["torso_height"] == 1.2
    assert params["torso_mass"] == 40.0
    assert params["thigh_length"] == 0.3 # Fallback default
