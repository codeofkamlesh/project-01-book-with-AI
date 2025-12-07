"""
Utility for generating draw.io compatible diagrams for the AI/Spec-Driven Robotics Book.
"""
from typing import Dict, List, Any, Optional
import xml.etree.ElementTree as ET
from xml.dom import minidom
import json


class DrawIOGenerator:
    """
    Generator for creating draw.io compatible diagrams.
    """
    def __init__(self):
        self.diagram_counter = 0

    def create_basic_diagram(self, title: str, width: int = 800, height: int = 600) -> str:
        """
        Create a basic draw.io diagram structure.
        """
        # Create the root mxfile element
        mxfile = ET.Element("mxfile", attrib={
            "host": "app.diagrams.net",
            "modified": "2025-12-07T00:00:00.000Z",
            "agent": "Mozilla/5.0",
            "etag": "example_etag",
            "version": "21.0.7",
            "type": "device"
        })

        # Create diagram element
        diagram = ET.SubElement(mxfile, "diagram", attrib={
            "name": title,
            "id": f"diagram_{self.diagram_counter}"
        })
        self.diagram_counter += 1

        # Create mxGraphModel
        graph_model = ET.SubElement(diagram, "mxGraphModel", attrib={
            "dx": str(width),
            "dy": str(height),
            "grid": "1",
            " gridSize": "10",
            " guides": "1",
            " tooltips": "1",
            " connect": "1",
            " arrows": "1",
            " fold": "1",
            " page": "1",
            " pageScale": "1",
            " pageWidth": str(width),
            " pageHeight": str(height),
            " background": "#ffffff",
            " math": "0",
            " shadow": "0"
        })

        # Create root for the diagram
        root = ET.SubElement(graph_model, "root")

        # Add default parent cells
        default_parent = ET.Element("mxCell", id="0")
        root.append(default_parent)

        default_layer = ET.Element("mxCell", id="1", parent="0", attrib={"label": title, "vertex": "1"})
        root.append(default_layer)

        # Convert to string and prettify
        rough_string = ET.tostring(root, encoding='unicode')
        diagram_content = f"""<mxGraphModel dx="{width}" dy="{height}" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="{width}" pageHeight="{height}" background="#ffffff" math="0" shadow="0">
{self.prettify_xml(rough_string)}
</mxGraphModel>"""

        full_diagram = f"""<mxfile host="app.diagrams.net" modified="2025-12-07T00:00:00.000Z" agent="Mozilla/5.0" etag="example_etag" version="21.0.7" type="device">
  <diagram name="{title}" id="diagram_{self.diagram_counter-1}">
{self.prettify_xml(diagram_content)}
  </diagram>
</mxfile>"""

        return full_diagram

    def prettify_xml(self, elem):
        """Return a pretty-printed XML string for the Element."""
        rough_string = ET.tostring(ET.fromstring(f"<root>{elem}</root>"), encoding='unicode')
        reparsed = minidom.parseString(rough_string)
        return '\n'.join([line for line in reparsed.toprettyxml(indent="  ").split('\n')[1:]])

    def create_ros2_architecture_diagram(self) -> str:
        """
        Create a ROS2 architecture diagram showing nodes, topics, services, etc.
        """
        diagram = self.create_basic_diagram("ROS2 Architecture", 1000, 800)

        # This would contain the actual ROS2 architecture diagram content
        # For now, we'll return the basic structure
        return diagram

    def create_simulation_workflow_diagram(self) -> str:
        """
        Create a simulation workflow diagram.
        """
        diagram = self.create_basic_diagram("Simulation Workflow", 1000, 800)

        # This would contain the actual simulation workflow diagram content
        return diagram

    def create_isaac_architecture_diagram(self) -> str:
        """
        Create an Isaac architecture diagram.
        """
        diagram = self.create_basic_diagram("Isaac Architecture", 1000, 800)

        # This would contain the actual Isaac architecture diagram content
        return diagram

    def create_vla_pipeline_diagram(self) -> str:
        """
        Create a VLA pipeline diagram showing vision-language-action flow.
        """
        diagram = self.create_basic_diagram("VLA Pipeline", 1000, 800)

        # This would contain the actual VLA pipeline diagram content
        return diagram

    def create_cross_model_integration_diagram(self) -> str:
        """
        Create a diagram showing integration between all four models.
        """
        diagram = self.create_basic_diagram("Cross-Model Integration", 1200, 800)

        # This would contain the actual integration diagram content
        return diagram

    def save_diagram(self, diagram_content: str, file_path: str):
        """
        Save the diagram to a file.
        """
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(diagram_content)