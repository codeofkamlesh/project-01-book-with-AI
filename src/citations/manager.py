"""
Citation management system for handling references to official documentation,
research papers, and authoritative sources.
"""
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from datetime import datetime
import json
import os


@dataclass
class Citation:
    """
    Represents a citation to a source.
    """
    id: str
    title: str
    url: Optional[str] = None
    authors: Optional[List[str]] = None
    year: Optional[int] = None
    journal: Optional[str] = None
    publisher: Optional[str] = None
    doi: Optional[str] = None
    source_type: Optional[str] = None  # 'documentation', 'paper', 'official', 'book', 'website'
    accessed_date: Optional[datetime] = None
    version: Optional[str] = None
    note: Optional[str] = None

    def __post_init__(self):
        if self.accessed_date is None:
            self.accessed_date = datetime.now()

    def format_apa(self) -> str:
        """
        Format the citation in APA style.
        """
        if self.authors:
            authors_str = ", ".join(self.authors)
            if len(self.authors) > 1:
                authors_str = f"{authors_str}"
        else:
            authors_str = "Unknown Author"

        if self.year:
            year_str = f"({self.year}). "
        else:
            year_str = "(n.d.). "

        title_part = f"{self.title}. "

        if self.journal:
            journal_part = f"*{self.journal}*, "
        else:
            journal_part = ""

        if self.publisher:
            publisher_part = f"{self.publisher}. "
        else:
            publisher_part = ""

        if self.url:
            url_part = f"Retrieved from {self.url}"
        else:
            url_part = ""

        return f"{authors_str}{year_str}{title_part}{journal_part}{publisher_part}{url_part}"

    def format_bibtex(self) -> str:
        """
        Format the citation in BibTeX style.
        """
        if self.source_type == "paper":
            entry_type = "article"
            extra_fields = f",\n    journal = {{{self.journal}}}" if self.journal else ""
        elif self.source_type == "book":
            entry_type = "book"
            extra_fields = f",\n    publisher = {{{self.publisher}}}" if self.publisher else ""
        else:
            entry_type = "misc"
            extra_fields = ""

        authors = " and ".join(self.authors) if self.authors else "Unknown Author"
        year = str(self.year) if self.year else "n.d."

        bibtex = f"@{entry_type}{{{self.id},\n" \
                 f"    title = {{{self.title}}},\n" \
                 f"    author = {{{authors}}},\n" \
                 f"    year = {{{year}}}{extra_fields}"

        if self.url:
            bibtex += f",\n    url = {{{self.url}}}"

        if self.doi:
            bibtex += f",\n    doi = {{{self.doi}}}"

        bibtex += "\n}"

        return bibtex


class CitationManager:
    """
    Manages citations for the book.
    """
    def __init__(self, citations_file: str = "citations.json"):
        self.citations_file = citations_file
        self.citations: Dict[str, Citation] = {}
        self.load_citations()

    def add_citation(self, citation: Citation):
        """
        Add a citation to the manager.
        """
        self.citations[citation.id] = citation

    def get_citation(self, citation_id: str) -> Optional[Citation]:
        """
        Get a citation by ID.
        """
        return self.citations.get(citation_id)

    def format_citation(self, citation_id: str, style: str = "apa") -> Optional[str]:
        """
        Format a citation in the specified style.
        """
        citation = self.get_citation(citation_id)
        if not citation:
            return None

        if style.lower() == "apa":
            return citation.format_apa()
        elif style.lower() == "bibtex":
            return citation.format_bibtex()
        else:
            return citation.format_apa()  # Default to APA

    def create_citation_from_doc(self, title: str, url: str, technology: str,
                                version: str = None, accessed_date: datetime = None) -> Citation:
        """
        Create a citation from documentation.
        """
        citation_id = f"{technology}_{hash(url) % 10000}"
        return Citation(
            id=citation_id,
            title=title,
            url=url,
            source_type="documentation",
            publisher=technology,
            version=version,
            accessed_date=accessed_date or datetime.now()
        )

    def create_citation_from_paper(self, title: str, authors: List[str],
                                  year: int, journal: str, doi: str = None) -> Citation:
        """
        Create a citation from a research paper.
        """
        citation_id = f"paper_{hash(title) % 10000}"
        return Citation(
            id=citation_id,
            title=title,
            authors=authors,
            year=year,
            journal=journal,
            doi=doi,
            source_type="paper"
        )

    def save_citations(self):
        """
        Save citations to the citations file.
        """
        citations_list = []
        for citation in self.citations.values():
            citation_dict = {
                'id': citation.id,
                'title': citation.title,
                'url': citation.url,
                'authors': citation.authors,
                'year': citation.year,
                'journal': citation.journal,
                'publisher': citation.publisher,
                'doi': citation.doi,
                'source_type': citation.source_type,
                'accessed_date': citation.accessed_date.isoformat() if citation.accessed_date else None,
                'version': citation.version,
                'note': citation.note
            }
            citations_list.append(citation_dict)

        with open(self.citations_file, 'w', encoding='utf-8') as f:
            json.dump(citations_list, f, indent=2, default=str)

    def load_citations(self):
        """
        Load citations from the citations file.
        """
        if not os.path.exists(self.citations_file):
            # Create an empty file if it doesn't exist
            with open(self.citations_file, 'w', encoding='utf-8') as f:
                json.dump([], f)
            return

        try:
            with open(self.citations_file, 'r', encoding='utf-8') as f:
                citations_list = json.load(f)

            for citation_data in citations_list:
                accessed_date = datetime.fromisoformat(citation_data['accessed_date']) if citation_data['accessed_date'] else None
                citation = Citation(
                    id=citation_data['id'],
                    title=citation_data['title'],
                    url=citation_data['url'],
                    authors=citation_data['authors'],
                    year=citation_data['year'],
                    journal=citation_data['journal'],
                    publisher=citation_data['publisher'],
                    doi=citation_data['doi'],
                    source_type=citation_data['source_type'],
                    accessed_date=accessed_date,
                    version=citation_data['version'],
                    note=citation_data['note']
                )
                self.citations[citation.id] = citation
        except (json.JSONDecodeError, KeyError, TypeError):
            # If there's an error loading, start with empty citations
            self.citations = {}

    def validate_citation_accessibility(self, citation_id: str) -> bool:
        """
        Validate that a cited URL is accessible (for documentation URLs).
        """
        import requests
        citation = self.get_citation(citation_id)
        if not citation or not citation.url:
            return False

        try:
            response = requests.head(citation.url, timeout=10)
            return response.status_code < 400
        except:
            return False

    def generate_bibliography(self, style: str = "apa") -> str:
        """
        Generate a bibliography in the specified style.
        """
        bibliography = []
        for citation_id in sorted(self.citations.keys()):
            formatted = self.format_citation(citation_id, style)
            if formatted:
                bibliography.append(formatted)

        return "\n\n".join(bibliography)


# Example usage and predefined citations
def get_common_robotics_citations() -> List[Citation]:
    """
    Get a list of common robotics citations referenced in the book.
    """
    citations = [
        Citation(
            id="ros2_docs_2022",
            title="ROS 2 Documentation",
            url="https://docs.ros.org/en/humble/",
            publisher="Open Robotics",
            year=2022,
            source_type="documentation",
            version="Humble Hawksbill",
            accessed_date=datetime.now()
        ),
        Citation(
            id="gazebo_docs_2023",
            title="Gazebo Simulation Documentation",
            url="https://gazebosim.org/docs/harmonic/",
            publisher="Open Robotics",
            year=2023,
            source_type="documentation",
            version="Harmonic",
            accessed_date=datetime.now()
        ),
        Citation(
            id="isaac_docs_2023",
            title="NVIDIA Isaac Documentation",
            url="https://docs.nvidia.com/isaac/",
            publisher="NVIDIA",
            year=2023,
            source_type="documentation",
            version="2023.1",
            accessed_date=datetime.now()
        ),
        Citation(
            id="rt1_paper_2022",
            title="RT-1: Robotics Transformer for Real-World Control at Scale",
            authors=["Brohan, Anthony", "Chebotar, Yevgen", "Duvall, Joseph", "Finn, Chelsea", "Fu, Keerthana", "Gopalakrishnan, Karol", "Herzog, Alex", "Jang, Jasmine", "Irpan, Alex", "Kappler, Daniel", "Mania, Hagen", "Mayya, Sridhar", "Morrison, Deirdre", "Nair, Suraj", "Pertsch, Karl", "Petrov, Michael", "Rodriguez, Ayzaan", "Salanta, Elena", "Scott, Katie", "Siegel, Alex", "Tan, Quan", "Tian, Alexander", "Tyler, Chelsea", "Vanhoucke, Vincent", "Xiao, Fei", "Xie, Ted", "Zhang, Ping", "Zeng, Andy", "Zhou, Eric"],
            year=2022,
            journal="arXiv preprint arXiv:2212.06817",
            source_type="paper",
            accessed_date=datetime.now()
        ),
        Citation(
            id="openvla_paper_2024",
            title="OpenVLA: An Open Vision-Language-Action Model",
            authors=["Kollar, Thomas", "Brohan, Anthony", "Kappler, Daniel", "Siegel, Alex", "Chen, Kanishka", "Jang, Jasmine", "Finn, Chelsea", "Zeng, Andy"],
            year=2024,
            journal="arXiv preprint arXiv:2406.09246",
            source_type="paper",
            accessed_date=datetime.now()
        )
    ]

    return citations


def initialize_citation_manager() -> CitationManager:
    """
    Initialize the citation manager with common robotics citations.
    """
    manager = CitationManager()

    # Add common robotics citations
    for citation in get_common_robotics_citations():
        manager.add_citation(citation)

    # Save the citations
    manager.save_citations()

    return manager