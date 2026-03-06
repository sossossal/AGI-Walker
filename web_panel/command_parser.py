"""
Simple Command Parser for AGI-Walker Agent.
Converts natural language commands into structured Robot Configuration JSON.
"""
import re
import json
from typing import Dict, Any, Optional

class CommandParser:
    """
    Parses agent commands to generate robot configurations.
    Current support:
    - Type selection (quadruped, hexapod, biped)
    - Basic parameter extraction (height, weight - future)
    """
    
    def parse(self, command: str) -> Dict[str, Any]:
        """
        Parse a text command and extract parameters for the Skills robot builder.
        Simulates an LLM intent understanding module.
        """
        cmd_lower = command.lower()
        
        # Default generated parameters
        skills_params = {
            "name": "ai_generated_robot",
            "type": "biped",
            "torso_height": 0.5,
            "torso_mass": 5.0,
            "thigh_length": 0.3,
            "shin_length": 0.3,
            "target_com_height": 0.22
        }

        # 1. Type extraction
        if "四足" in cmd_lower or "quadruped" in cmd_lower or "狗" in cmd_lower:
            skills_params["type"] = "quadruped"
            skills_params["name"] = "ai_quadruped"
            skills_params["target_com_height"] = 0.15
        elif "双足" in cmd_lower or "biped" in cmd_lower or "人" in cmd_lower:
            skills_params["type"] = "biped"
            skills_params["name"] = "ai_biped"
            skills_params["target_com_height"] = 0.25

        # 2. Dimensions extraction (Regex simulation)
        height_match = re.search(r'高(?:度)?.*?([\d\.]+)\s*[m米]', cmd_lower)
        if height_match:
            skills_params["torso_height"] = float(height_match.group(1))

        mass_match = re.search(r'重(?:量)?.*?([\d\.]+)\s*[k千]', cmd_lower)
        if mass_match:
            skills_params["torso_mass"] = float(mass_match.group(1))
            
        leg_match = re.search(r'大腿.*?([\d\.]+)\s*[m米]', cmd_lower)
        if leg_match:
            skills_params["thigh_length"] = float(leg_match.group(1))

        # Build response
        config = {
            "parts": [], # Legacy requirement fallback
            "connections": [],
            "metadata": {
                "source_command": command,
                "type": skills_params["type"],
                "is_from_skills_llm": True
            },
            "skills_params": skills_params
        }
            
        return config
