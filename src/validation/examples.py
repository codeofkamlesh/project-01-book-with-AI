"""
Example validation scripts for validating code examples in the book.
"""
import subprocess
import tempfile
import os
import sys
from typing import Dict, List, Any, Optional
from pathlib import Path


class ExampleValidator:
    """
    Validator for code examples in the book.
    """
    def __init__(self):
        self.validation_results = {}

    def validate_ros2_example(self, code: str, requirements: List[str] = None) -> Dict[str, Any]:
        """
        Validate a ROS2 Python example.
        """
        result = {
            'success': False,
            'errors': [],
            'warnings': [],
            'output': '',
            'execution_time': 0.0
        }

        # Check basic syntax first
        try:
            compile(code, '<string>', 'exec')
        except SyntaxError as e:
            result['errors'].append(f"Syntax error: {str(e)}")
            return result

        # Create a temporary file with the code
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
            f.write(code)
            temp_file = f.name

        try:
            # Run the code in a subprocess to avoid side effects
            # For ROS2 examples, we'd normally run with proper ROS2 environment
            completed_process = subprocess.run(
                [sys.executable, temp_file],
                capture_output=True,
                text=True,
                timeout=30  # 30 second timeout
            )

            result['output'] = completed_process.stdout
            if completed_process.stderr:
                result['errors'].append(completed_process.stderr)

            result['success'] = completed_process.returncode == 0
        except subprocess.TimeoutExpired:
            result['errors'].append("Execution timed out after 30 seconds")
        except Exception as e:
            result['errors'].append(f"Execution error: {str(e)}")
        finally:
            # Clean up temporary file
            os.unlink(temp_file)

        return result

    def validate_python_example(self, code: str, requirements: List[str] = None) -> Dict[str, Any]:
        """
        Validate a general Python example.
        """
        result = {
            'success': False,
            'errors': [],
            'warnings': [],
            'output': '',
            'execution_time': 0.0
        }

        # Check basic syntax first
        try:
            compile(code, '<string>', 'exec')
        except SyntaxError as e:
            result['errors'].append(f"Syntax error: {str(e)}")
            return result

        # Create a temporary file with the code
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
            f.write(code)
            temp_file = f.name

        try:
            # Run the code in a subprocess
            completed_process = subprocess.run(
                [sys.executable, temp_file],
                capture_output=True,
                text=True,
                timeout=30  # 30 second timeout
            )

            result['output'] = completed_process.stdout
            if completed_process.stderr:
                result['errors'].append(completed_process.stderr)

            result['success'] = completed_process.returncode == 0
        except subprocess.TimeoutExpired:
            result['errors'].append("Execution timed out after 30 seconds")
        except Exception as e:
            result['errors'].append(f"Execution error: {str(e)}")
        finally:
            # Clean up temporary file
            os.unlink(temp_file)

        return result

    def validate_launch_file(self, file_path: str) -> Dict[str, Any]:
        """
        Validate a ROS2 launch file.
        """
        result = {
            'success': False,
            'errors': [],
            'warnings': [],
            'validation_details': ''
        }

        try:
            # Check if file exists
            if not os.path.exists(file_path):
                result['errors'].append(f"Launch file does not exist: {file_path}")
                return result

            # Check file extension
            if not file_path.endswith(('.py', '.xml', '.yaml')):
                result['errors'].append(f"Invalid launch file extension: {file_path}")
                return result

            # For Python launch files, check syntax
            if file_path.endswith('.py'):
                with open(file_path, 'r', encoding='utf-8') as f:
                    code = f.read()
                try:
                    compile(code, file_path, 'exec')
                    result['success'] = True
                    result['validation_details'] = "Launch file syntax is valid"
                except SyntaxError as e:
                    result['errors'].append(f"Launch file syntax error: {str(e)}")
            else:
                # For XML/YAML launch files, we could add more specific validation
                result['success'] = True
                result['validation_details'] = "Launch file exists and has valid extension"

        except Exception as e:
            result['errors'].append(f"Error validating launch file: {str(e)}")

        return result

    def validate_simulation_world(self, file_path: str) -> Dict[str, Any]:
        """
        Validate a simulation world file (SDF, URDF, etc.).
        """
        result = {
            'success': False,
            'errors': [],
            'warnings': [],
            'validation_details': ''
        }

        try:
            # Check if file exists
            if not os.path.exists(file_path):
                result['errors'].append(f"World file does not exist: {file_path}")
                return result

            # Check file extension
            valid_extensions = ['.sdf', '.urdf', '.world', '.xml']
            file_ext = os.path.splitext(file_path)[1].lower()
            if file_ext not in valid_extensions:
                result['errors'].append(f"Invalid world file extension: {file_path}")
                return result

            # Check file size (reasonable limit)
            file_size = os.path.getsize(file_path)
            if file_size > 50 * 1024 * 1024:  # 50MB limit
                result['warnings'].append(f"World file is large: {file_size / (1024*1024):.2f} MB")

            # For SDF/URDF files, we could add XML validation
            if file_ext in ['.sdf', '.urdf', '.world', '.xml']:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Basic XML check - ensure it starts with <?xml or has root element
                content_stripped = content.strip()
                if content_stripped.startswith('<?xml') or '<sdf' in content_stripped or '<robot' in content_stripped:
                    result['success'] = True
                    result['validation_details'] = f"World file has valid XML structure, size: {file_size} bytes"
                else:
                    result['errors'].append("File does not appear to be valid XML")
            else:
                result['success'] = True
                result['validation_details'] = f"World file exists, size: {file_size} bytes"

        except Exception as e:
            result['errors'].append(f"Error validating world file: {str(e)}")

        return result

    def validate_isaac_pipeline(self, pipeline_config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate an Isaac pipeline configuration.
        """
        result = {
            'success': False,
            'errors': [],
            'warnings': [],
            'validation_details': ''
        }

        required_keys = ['name', 'components', 'graph']
        for key in required_keys:
            if key not in pipeline_config:
                result['errors'].append(f"Missing required key: {key}")

        if 'components' in pipeline_config:
            components = pipeline_config['components']
            if not isinstance(components, list) or len(components) == 0:
                result['errors'].append("Components must be a non-empty list")

        if 'graph' in pipeline_config:
            graph = pipeline_config['graph']
            if not isinstance(graph, dict) or 'nodes' not in graph:
                result['errors'].append("Graph must be a dictionary with 'nodes' key")

        if len(result['errors']) == 0:
            result['success'] = True
            result['validation_details'] = "Isaac pipeline configuration is valid"

        return result

    def validate_vla_example(self, code: str, input_data: Any) -> Dict[str, Any]:
        """
        Validate a VLA (Vision-Language-Action) example.
        """
        result = {
            'success': False,
            'errors': [],
            'warnings': [],
            'output': '',
            'execution_time': 0.0
        }

        # Check basic syntax first
        try:
            compile(code, '<string>', 'exec')
        except SyntaxError as e:
            result['errors'].append(f"Syntax error: {str(e)}")
            return result

        # Create a temporary file with the code
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
            f.write(code)
            temp_file = f.name

        try:
            # Run the code in a subprocess
            completed_process = subprocess.run(
                [sys.executable, temp_file],
                capture_output=True,
                text=True,
                timeout=60  # 60 second timeout for VLA examples
            )

            result['output'] = completed_process.stdout
            if completed_process.stderr:
                result['errors'].append(completed_process.stderr)

            result['success'] = completed_process.returncode == 0
        except subprocess.TimeoutExpired:
            result['errors'].append("Execution timed out after 60 seconds")
        except Exception as e:
            result['errors'].append(f"Execution error: {str(e)}")
        finally:
            # Clean up temporary file
            os.unlink(temp_file)

        return result


# Example usage and test functions
def test_example_validator():
    """
    Test function for the example validator.
    """
    validator = ExampleValidator()

    # Test Python code validation
    test_code = """
print("Hello, World!")
x = 1 + 1
print(f"1 + 1 = {x}")
"""

    result = validator.validate_python_example(test_code)
    print("Python example validation result:", result)

    return result


if __name__ == "__main__":
    test_example_validator()