"""
Research collection framework for gathering robotics documentation, papers, and official sources.
"""
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from datetime import datetime
import json
import os
import requests
from urllib.parse import urlparse


@dataclass
class ResearchSource:
    """
    Represents a research source (documentation, paper, official source).
    """
    id: str
    title: str
    url: str
    source_type: str  # 'documentation', 'paper', 'official', 'tutorial'
    technology: str  # 'ROS2', 'Gazebo', 'Unity', 'Isaac', 'VLA'
    version: Optional[str] = None
    access_date: datetime = None
    content_summary: str = ""
    citation: str = ""
    verified: bool = False

    def __post_init__(self):
        if self.access_date is None:
            self.access_date = datetime.now()


class ResearchCollector:
    """
    Framework for collecting and organizing research materials.
    """
    def __init__(self, research_dir: str = "research"):
        self.research_dir = research_dir
        self.sources: List[ResearchSource] = []
        self.technology_dirs = {
            'ROS2': os.path.join(research_dir, 'ros2'),
            'Gazebo': os.path.join(research_dir, 'simulation'),
            'Unity': os.path.join(research_dir, 'simulation'),
            'Isaac': os.path.join(research_dir, 'isaac'),
            'VLA': os.path.join(research_dir, 'vla')
        }

        # Create research directories if they don't exist
        for dir_path in self.technology_dirs.values():
            os.makedirs(dir_path, exist_ok=True)

    def add_source(self, source: ResearchSource):
        """
        Add a research source to the collection.
        """
        self.sources.append(source)

    def collect_ros2_documentation(self) -> List[ResearchSource]:
        """
        Collect ROS2 official documentation sources.
        """
        sources = []
        ros2_docs = [
            {
                'title': 'ROS2 Humble Documentation',
                'url': 'https://docs.ros.org/en/humble/',
                'version': 'Humble Hawksbill',
                'summary': 'Official ROS2 documentation for Humble Hawksbill distribution'
            },
            {
                'title': 'ROS2 Tutorials',
                'url': 'https://docs.ros.org/en/humble/Tutorials.html',
                'version': 'Humble Hawksbill',
                'summary': 'Step-by-step tutorials for ROS2 concepts and usage'
            },
            {
                'title': 'ROS2 Design',
                'url': 'https://design.ros2.org/',
                'summary': 'Design documents and rationale for ROS2 architecture'
            }
        ]

        for doc in ros2_docs:
            source = ResearchSource(
                id=f"ros2_{len(self.sources)}",
                title=doc['title'],
                url=doc['url'],
                source_type='documentation',
                technology='ROS2',
                version=doc.get('version'),
                content_summary=doc['summary'],
                citation=f"ROS.org. {doc['title']}. Retrieved {datetime.now().strftime('%Y-%m-%d')}"
            )
            sources.append(source)
            self.add_source(source)

        return sources

    def collect_simulation_documentation(self) -> List[ResearchSource]:
        """
        Collect Gazebo and Unity simulation documentation.
        """
        sources = []
        sim_docs = [
            {
                'title': 'Gazebo Harmonic Documentation',
                'url': 'https://gazebosim.org/docs/harmonic',
                'version': 'Harmonic',
                'technology': 'Gazebo',
                'summary': 'Official Gazebo Harmonic documentation'
            },
            {
                'title': 'Unity Robotics Hub',
                'url': 'https://github.com/Unity-Technologies/Unity-Robotics-Hub',
                'technology': 'Unity',
                'summary': 'Unity tools and examples for robotics development'
            }
        ]

        for doc in sim_docs:
            source = ResearchSource(
                id=f"sim_{len(self.sources)}",
                title=doc['title'],
                url=doc['url'],
                source_type='documentation',
                technology=doc['technology'],
                version=doc.get('version'),
                content_summary=doc['summary'],
                citation=f"{doc['technology']}. {doc['title']}. Retrieved {datetime.now().strftime('%Y-%m-%d')}"
            )
            sources.append(source)
            self.add_source(source)

        return sources

    def collect_isaac_documentation(self) -> List[ResearchSource]:
        """
        Collect NVIDIA Isaac documentation and resources.
        """
        sources = []
        isaac_docs = [
            {
                'title': 'NVIDIA Isaac Documentation',
                'url': 'https://docs.nvidia.com/isaac/',
                'summary': 'Official NVIDIA Isaac platform documentation'
            },
            {
                'title': 'Isaac Sim Documentation',
                'url': 'https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html',
                'summary': 'NVIDIA Isaac Sim simulation environment documentation'
            },
            {
                'title': 'Isaac ROS GEMs',
                'url': 'https://github.com/NVIDIA-ISAAC-ROS',
                'summary': 'GPU-accelerated ROS packages for perception and navigation'
            }
        ]

        for doc in isaac_docs:
            source = ResearchSource(
                id=f"isaac_{len(self.sources)}",
                title=doc['title'],
                url=doc['url'],
                source_type='documentation',
                technology='Isaac',
                content_summary=doc['summary'],
                citation=f"NVIDIA. {doc['title']}. Retrieved {datetime.now().strftime('%Y-%m-%d')}"
            )
            sources.append(source)
            self.add_source(source)

        return sources

    def collect_vla_research(self) -> List[ResearchSource]:
        """
        Collect VLA (Vision-Language-Action) research papers and documentation.
        """
        sources = []
        vla_docs = [
            {
                'title': 'RT-1: Robotics Transformer 1',
                'url': 'https://arxiv.org/abs/2212.06817',
                'source_type': 'paper',
                'summary': 'Learning Unstructured Task Specifications from Language and Human Demonstrations'
            },
            {
                'title': 'OpenVLA: Open Vision-Language-Action Models',
                'url': 'https://github.com/openvla/openvla',
                'summary': 'Open-source VLA models for robot manipulation'
            }
        ]

        for doc in vla_docs:
            source = ResearchSource(
                id=f"vla_{len(self.sources)}",
                title=doc['title'],
                url=doc['url'],
                source_type=doc.get('source_type', 'research'),
                technology='VLA',
                content_summary=doc['summary'],
                citation=f"Various Authors. {doc['title']}. Retrieved {datetime.now().strftime('%Y-%m-%d')}"
            )
            sources.append(source)
            self.add_source(source)

        return sources

    def collect_all_research(self) -> Dict[str, List[ResearchSource]]:
        """
        Collect all research materials for the four models.
        """
        results = {
            'ROS2': self.collect_ros2_documentation(),
            'Simulation': self.collect_simulation_documentation(),
            'Isaac': self.collect_isaac_documentation(),
            'VLA': self.collect_vla_research()
        }
        return results

    def save_sources(self, technology: str = None):
        """
        Save collected sources to JSON files organized by technology.
        """
        if technology:
            # Save sources for specific technology
            tech_sources = [s for s in self.sources if s.technology == technology]
            file_path = os.path.join(self.research_dir, technology.lower(), 'sources.json')
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump([self.source_to_dict(s) for s in tech_sources], f, indent=2, default=str)
        else:
            # Save all sources organized by technology
            for tech in self.technology_dirs:
                tech_sources = [s for s in self.sources if s.technology == tech]
                file_path = os.path.join(self.technology_dirs[tech], 'sources.json')
                os.makedirs(os.path.dirname(file_path), exist_ok=True)
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump([self.source_to_dict(s) for s in tech_sources], f, indent=2, default=str)

    def source_to_dict(self, source: ResearchSource) -> Dict[str, Any]:
        """
        Convert ResearchSource to dictionary for JSON serialization.
        """
        return {
            'id': source.id,
            'title': source.title,
            'url': source.url,
            'source_type': source.source_type,
            'technology': source.technology,
            'version': source.version,
            'access_date': source.access_date.isoformat() if source.access_date else None,
            'content_summary': source.content_summary,
            'citation': source.citation,
            'verified': source.verified
        }

    def verify_source_accessibility(self, source: ResearchSource) -> bool:
        """
        Verify that a source is accessible (for documentation URLs).
        """
        try:
            response = requests.head(source.url, timeout=10)
            is_accessible = response.status_code < 400
            source.verified = is_accessible
            return is_accessible
        except:
            source.verified = False
            return False