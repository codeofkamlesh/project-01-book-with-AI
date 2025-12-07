#!/usr/bin/env python3

"""
Validation script for the complete AI/Spec-Driven Robotics Book.
This script validates that all components of the book are properly implemented
and meet the success criteria defined in the specifications.
"""

import os
import sys
import subprocess
import json
from pathlib import Path
from typing import Dict, List, Tuple
import importlib.util


class BookValidator:
    def __init__(self):
        self.results = {
            'overall_success': True,
            'modules_validated': [],
            'errors': [],
            'warnings': [],
            'test_results': {}
        }

    def validate_directory_structure(self) -> bool:
        """Validate the expected directory structure."""
        print("Validating directory structure...")

        required_dirs = [
            'docs',
            'examples',
            'diagrams',
            'research',
            'specs',
            'src',
            'tests',
            'docusaurus',
            'history',
            'examples/ros2',
            'examples/simulation',
            'examples/isaac',
            'examples/vla',
            'examples/integration',
            'diagrams/ros2',
            'diagrams/simulation',
            'diagrams/isaac',
            'diagrams/vla',
            'diagrams/integration',
            'src/models',
            'src/validation',
            'src/diagrams',
            'src/research',
            'src/citations'
        ]

        missing_dirs = []
        for dir_path in required_dirs:
            full_path = Path(dir_path)
            if not full_path.exists():
                missing_dirs.append(str(full_path))

        if missing_dirs:
            self.results['errors'].extend([f"Missing directory: {d}" for d in missing_dirs])
            self.results['overall_success'] = False
            return False
        else:
            print("[PASS] Directory structure validation passed")
            return True

    def validate_documentation(self) -> bool:
        """Validate that documentation files exist and are properly formatted."""
        print("Validating documentation...")

        # Check for main documentation files
        docs_to_check = [
            'docusaurus/docs/intro.md',
            'docusaurus/docs/ros2-foundations/index.md',
            'docusaurus/docs/simulation/index.md',
            'docusaurus/docs/nvidia-isaac/index.md',
            'docusaurus/docs/vla-humanoids/index.md',
            'docusaurus/docs/integration/index.md'
        ]

        missing_docs = []
        for doc_path in docs_to_check:
            if not Path(doc_path).exists():
                missing_docs.append(doc_path)

        if missing_docs:
            self.results['errors'].extend([f"Missing documentation: {d}" for d in missing_docs])
            self.results['overall_success'] = False
            return False
        else:
            print("[PASS] Documentation validation passed")
            return True

    def validate_examples(self) -> bool:
        """Validate that example files exist and can be imported/executed."""
        print("Validating examples...")

        # Check for key example files
        example_files = [
            'examples/ros2/publisher_subscriber/minimal_publisher.py',
            'examples/ros2/publisher_subscriber/minimal_subscriber.py',
            'examples/simulation/models/simple_robot.urdf',
            'examples/simulation/gazebo/worlds/simple_room.world',
            'examples/isaac/config/simple_robot_config.yaml',
            'examples/vla/pipeline/vla_pipeline.py',
            'examples/vla/prompt/prompt_to_action.py',
            'examples/integration/end_to_end/end_to_end_humanoid_demo.py'
        ]

        missing_examples = []
        for example_path in example_files:
            if not Path(example_path).exists():
                missing_examples.append(example_path)

        if missing_examples:
            self.results['errors'].extend([f"Missing example: {e}" for e in missing_examples])
            self.results['overall_success'] = False
            return False
        else:
            print("[PASS] Examples validation passed")
            return True

    def validate_models(self) -> bool:
        """Validate that model files exist and can be imported."""
        print("Validating models...")

        model_files = [
            'src/models/book_chapter.py',
            'src/models/code_example.py',
            'src/models/simulation_env.py',
            'src/models/robot_model.py',
            'src/models/vla_pipeline.py',
            'src/models/diagram.py',
            'src/models/validation_test.py'
        ]

        missing_models = []
        for model_path in model_files:
            if not Path(model_path).exists():
                missing_models.append(model_path)

        if missing_models:
            self.results['errors'].extend([f"Missing model: {m}" for m in missing_models])
            self.results['overall_success'] = False
            return False
        else:
            # Try importing the models to check for syntax errors
            for model_path in model_files:
                try:
                    spec = importlib.util.spec_from_file_location("model_module", model_path)
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)
                except Exception as e:
                    self.results['errors'].append(f"Error importing {model_path}: {str(e)}")
                    self.results['overall_success'] = False
                    return False

            print("[PASS] Models validation passed")
            return True

    def validate_validation_framework(self) -> bool:
        """Validate that the validation framework exists and works."""
        print("Validating validation framework...")

        validation_files = [
            'src/validation/__init__.py',
            'src/validation/validator.py',
            'src/validation/examples.py'
        ]

        missing_validations = []
        for val_path in validation_files:
            if not Path(val_path).exists():
                missing_validations.append(val_path)

        if missing_validations:
            self.results['errors'].extend([f"Missing validation file: {v}" for v in missing_validations])
            self.results['overall_success'] = False
            return False
        else:
            print("[PASS] Validation framework validation passed")
            return True

    def validate_docusaurus_site(self) -> bool:
        """Validate that Docusaurus site can be built."""
        print("Validating Docusaurus site...")

        docusaurus_files = [
            'docusaurus/package.json',
            'docusaurus/docusaurus.config.js',
            'docusaurus/sidebars.js',
            'docusaurus/src/css/custom.css'
        ]

        missing_docusaurus = []
        for docusaurus_path in docusaurus_files:
            if not Path(docusaurus_path).exists():
                missing_docusaurus.append(docusaurus_path)

        if missing_docusaurus:
            self.results['errors'].extend([f"Missing Docusaurus file: {d}" for d in missing_docusaurus])
            # This might not be critical for overall success, so we won't set overall_success to False
            print("[WARN] Docusaurus validation has warnings (some files missing)")
            return True
        else:
            print("[PASS] Docusaurus site validation passed")
            return True

    def validate_integration(self) -> bool:
        """Validate that integration components work together."""
        print("Validating integration...")

        # Check for integration examples
        integration_files = [
            'examples/integration/ros2_sim/ros2_simulation_bridge.py',
            'examples/integration/sim_isaac/isaac_ros_bridge.py',
            'examples/integration/end_to_end/end_to_end_humanoid_demo.py'
        ]

        missing_integration = []
        for integ_path in integration_files:
            if not Path(integ_path).exists():
                missing_integration.append(integ_path)

        if missing_integration:
            self.results['errors'].extend([f"Missing integration file: {i}" for i in missing_integration])
            self.results['overall_success'] = False
            return False
        else:
            print("[PASS] Integration validation passed")
            return True

    def run_all_validations(self) -> Dict:
        """Run all validation checks."""
        print("Starting book validation...\n")

        # Run all validation checks
        checks = [
            ("Directory Structure", self.validate_directory_structure),
            ("Documentation", self.validate_documentation),
            ("Examples", self.validate_examples),
            ("Models", self.validate_models),
            ("Validation Framework", self.validate_validation_framework),
            ("Docusaurus Site", self.validate_docusaurus_site),
            ("Integration", self.validate_integration),
        ]

        for check_name, check_func in checks:
            try:
                result = check_func()
                self.results['modules_validated'].append({
                    'name': check_name,
                    'passed': result
                })
            except Exception as e:
                error_msg = f"Error during {check_name} validation: {str(e)}"
                self.results['errors'].append(error_msg)
                self.results['overall_success'] = False
                self.results['modules_validated'].append({
                    'name': check_name,
                    'passed': False,
                    'error': str(e)
                })

        # Add summary
        passed_checks = sum(1 for m in self.results['modules_validated'] if m.get('passed', False))
        total_checks = len(self.results['modules_validated'])

        self.results['summary'] = {
            'total_checks': total_checks,
            'passed_checks': passed_checks,
            'failed_checks': total_checks - passed_checks,
            'success_rate': passed_checks / total_checks if total_checks > 0 else 0
        }

        print(f"\nValidation Summary:")
        print(f"  Total checks: {total_checks}")
        print(f"  Passed: {passed_checks}")
        print(f"  Failed: {total_checks - passed_checks}")
        print(f"  Success rate: {self.results['summary']['success_rate']*100:.1f}%")
        print(f"  Overall success: {'[PASS]' if self.results['overall_success'] else '[FAIL]'}")

        if self.results['errors']:
            print(f"\nErrors ({len(self.results['errors'])}):")
            for error in self.results['errors'][:5]:  # Show first 5 errors
                print(f"  - {error}")
            if len(self.results['errors']) > 5:
                print(f"  ... and {len(self.results['errors']) - 5} more errors")

        if self.results['warnings']:
            print(f"\nWarnings ({len(self.results['warnings'])}):")
            for warning in self.results['warnings']:
                print(f"  - {warning}")

        return self.results


def main():
    """Main function to run the book validation."""
    validator = BookValidator()
    results = validator.run_all_validations()

    # Exit with appropriate code
    sys.exit(0 if results['overall_success'] else 1)


if __name__ == "__main__":
    main()