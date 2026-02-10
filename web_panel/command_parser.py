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
        Parse a text command and return a Robot Configuration.
        
        Args:
            command: Natural language command string
            
        Returns:
            Dict containing 'parts' and 'connections' keys, or error info.
        """
        cmd_lower = command.lower()
        
        # Default templates
        config = {
            "parts": [],
            "connections": [],
            "metadata": {
                "source_command": command,
                "type": "unknown"
            }
        }
        
        # 1. Determine Robot Type
        if "quadruped" in cmd_lower or "四足" in cmd_lower:
            config = self._create_quadruped_template()
            config["metadata"]["type"] = "quadruped"
        elif "hexapod" in cmd_lower or "六足" in cmd_lower:
            config = self._create_hexapod_template()
            config["metadata"]["type"] = "hexapod"
        elif "biped" in cmd_lower or "双足" in cmd_lower:
            config = self._create_biped_template()
            config["metadata"]["type"] = "biped"
        else:
            # Default to simple arm if unknown or ask for clarification (here we default to simple test bot)
            config = self._create_simple_bot_template()
            config["metadata"]["type"] = "simple_bot"
            
        return config

    def _create_simple_bot_template(self):
        """Creates a simple single-motor test bot"""
        return {
            "parts": [
                {"id": "base", "type": "structure", "model": "Base_Block", "position": [0, 0, 0]},
                {"id": "motor_1", "type": "motor", "model": "XM430-W350", "position": [0, 0.1, 0]}
            ],
            "connections": [
                {"from": "base", "to": "motor_1"}
            ],
            "metadata": {}
        }

    def _create_quadruped_template(self):
        """Creates a basic quadruped structure"""
        # Simplified representation for the MVP
        parts = [{"id": "body", "type": "structure", "model": "Main_Body", "position": [0, 0, 0.2]}]
        connections = []
        
        # 4 legs
        legs = ["fl", "fr", "bl", "br"]
        offsets = [[0.1, 0.1], [0.1, -0.1], [-0.1, 0.1], [-0.1, -0.1]]
        
        for i, leg in enumerate(legs):
            # Hip Motor
            motor_id = f"motor_{leg}_hip"
            parts.append({
                "id": motor_id, 
                "type": "motor", 
                "model": "XM430-W350", 
                "position": [offsets[i][0], offsets[i][1], 0.2]
            })
            connections.append({"from": "body", "to": motor_id})
            
            # Leg segment
            leg_id = f"leg_{leg}"
            parts.append({
                "id": leg_id,
                "type": "structure",
                "model": "Leg_Rod",
                "position": [offsets[i][0], offsets[i][1], 0.1] # Simplified pos
            })
            connections.append({"from": motor_id, "to": leg_id})
            
        return {"parts": parts, "connections": connections, "metadata": {}}

    def _create_hexapod_template(self):
        """Creates a basic hexapod structure"""
        parts = [{"id": "body", "type": "structure", "model": "Hex_Body", "position": [0, 0, 0.1]}]
        connections = []
        # Implement placeholder
        return {"parts": parts, "connections": connections, "metadata": {}}

    def _create_biped_template(self):
        """Creates a basic biped structure"""
        parts = [{"id": "pelvis", "type": "structure", "model": "Pelvis", "position": [0, 0, 0.5]}]
        connections = []
        # Implement placeholder
        return {"parts": parts, "connections": connections, "metadata": {}}
