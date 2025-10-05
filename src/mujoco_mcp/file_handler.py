#!/usr/bin/env python3
"""
File handling utilities for download/upload operations
Handles XML and Python script download/upload with validation
"""

import os
import tempfile
import xml.etree.ElementTree as ET
from typing import Dict, Any, Optional, Tuple
import logging
import uuid
from datetime import datetime
import mujoco

logger = logging.getLogger(__name__)

class FileHandler:
    """Handle file operations for XML and Python scripts"""
    
    def __init__(self, temp_dir: Optional[str] = None):
        self.temp_dir = temp_dir or tempfile.gettempdir()
        self.uploaded_files = {}  # Track uploaded files
        
    def validate_xml(self, xml_content: str) -> Dict[str, Any]:
        """Validate XML content for MuJoCo compatibility"""
        try:
            # Basic validation
            if not xml_content.strip():
                return {"valid": False, "error": "Empty XML content"}
            
            # Parse XML structure
            try:
                root = ET.fromstring(xml_content)
                if root.tag != "mujoco":
                    return {"valid": False, "error": "Not a valid MuJoCo XML (root tag must be 'mujoco')"}
            except ET.ParseError as e:
                return {"valid": False, "error": f"XML parse error: {str(e)}"}
            
            # Validate with MuJoCo
            try:
                with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as tmp:
                    tmp.write(xml_content)
                    tmp_path = tmp.name
                
                try:
                    model = mujoco.MjModel.from_xml_path(tmp_path)
                    result = {
                        "valid": True,
                        "model_info": {
                            "n_bodies": model.nbody,
                            "n_joints": model.njnt,
                            "n_actuators": model.nu,
                            "n_geoms": model.ngeom,
                            "xml_size": len(xml_content)
                        }
                    }
                    return result
                finally:
                    os.unlink(tmp_path)
                    
            except Exception as e:
                return {"valid": False, "error": f"MuJoCo validation failed: {str(e)}"}
                
        except Exception as e:
            return {"valid": False, "error": f"Validation error: {str(e)}"}
    
    def generate_python_script(self, model_xml: str, scene_name: str, include_viewer: bool = True) -> str:
        """Generate Python script that recreates the simulation"""
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        script = f'''#!/usr/bin/env python3
"""
Auto-generated MuJoCo simulation script
Generated on: {timestamp}
Scene: {scene_name}
"""

import mujoco
import numpy as np
{f"import mujoco.viewer" if include_viewer else ""}
import tempfile
import os

def create_scene():
    """Create the MuJoCo scene"""
    xml_content = """{model_xml}"""
    
    # Create temporary file for the XML
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as tmp:
        tmp.write(xml_content)
        xml_path = tmp.name
    
    try:
        # Load the model
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        
        print(f"✅ Loaded model: {{model.nbody}} bodies, {{model.njnt}} joints")
        return model, data, xml_path
        
    except Exception as e:
        print(f"❌ Failed to load model: {{e}}")
        os.unlink(xml_path)
        raise

def run_simulation():
    """Run the simulation"""
    model, data, xml_path = create_scene()
    
    try:
        {f'''# Launch viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("🚀 Viewer launched. Press Ctrl+C to exit.")
            
            # Simple simulation loop
            while viewer.is_running():
                # Step simulation
                mujoco.mj_step(model, data)
                
                # Sync viewer
                viewer.sync()
                
                # Small delay to prevent excessive CPU usage
                import time
                time.sleep(0.001)
    ''' if include_viewer else '''# Run headless simulation
        print("🚀 Running headless simulation...")
        
        # Run for 1000 steps
        for i in range(1000):
            mujoco.mj_step(model, data)
            
            if i % 100 == 0:
                print(f"Step {{i}}: time = {{data.time:.3f}}")
        
        print("✅ Simulation completed")
    '''}
    finally:
        # Cleanup
        os.unlink(xml_path)

if __name__ == "__main__":
    run_simulation()
'''
        return script
    
    def save_upload_file(self, content: str, file_type: str, name: Optional[str] = None) -> str:
        """Save uploaded file content and return file ID"""
        file_id = str(uuid.uuid4())
        timestamp = datetime.now().isoformat()
        
        # Generate filename if not provided
        if not name:
            ext = "xml" if file_type == "xml" else "py"
            name = f"uploaded_{file_type}_{file_id[:8]}.{ext}"
        
        # Store file info
        self.uploaded_files[file_id] = {
            "content": content,
            "type": file_type,
            "name": name,
            "timestamp": timestamp,
            "size": len(content)
        }
        
        logger.info(f"Saved uploaded {file_type} file: {name} (ID: {file_id})")
        return file_id
    
    def get_file_content(self, file_id: str) -> Optional[Dict[str, Any]]:
        """Get uploaded file content by ID"""
        return self.uploaded_files.get(file_id)
    
    def list_uploaded_files(self) -> Dict[str, Any]:
        """List all uploaded files"""
        return {
            "total_files": len(self.uploaded_files),
            "files": [
                {
                    "id": file_id,
                    "name": info["name"],
                    "type": info["type"],
                    "size": info["size"],
                    "timestamp": info["timestamp"]
                }
                for file_id, info in self.uploaded_files.items()
            ]
        }
    
    def validate_python_script(self, script_content: str, safe_mode: bool = True) -> Dict[str, Any]:
        """Validate Python script for safety and syntax"""
        try:
            # Check for syntax errors
            compile(script_content, '<uploaded_script>', 'exec')
            
            if safe_mode:
                # Check for potentially dangerous operations
                dangerous_imports = [
                    'subprocess', 'os.system', 'eval', 'exec', 'open',
                    '__import__', 'getattr', 'setattr', 'delattr',
                    'globals', 'locals', 'vars'
                ]
                
                dangerous_found = []
                for dangerous in dangerous_imports:
                    if dangerous in script_content:
                        dangerous_found.append(dangerous)
                
                if dangerous_found:
                    return {
                        "valid": False,
                        "error": f"Potentially dangerous operations found: {', '.join(dangerous_found)}",
                        "suggestion": "Consider running in non-safe mode if you trust this script"
                    }
            
            return {"valid": True, "safe_mode": safe_mode}
            
        except SyntaxError as e:
            return {"valid": False, "error": f"Python syntax error: {str(e)}"}
        except Exception as e:
            return {"valid": False, "error": f"Script validation error: {str(e)}"}