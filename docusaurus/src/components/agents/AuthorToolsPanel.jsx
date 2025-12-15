import React, { useState, useContext } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { API_ENDPOINTS } from '../../utils/apiConfig';

const AuthorToolsPanel = () => {
  const { user, isAuthenticated } = useAuth();
  const [activeTab, setActiveTab] = useState('ros2');
  const [loading, setLoading] = useState(false);
  const [results, setResults] = useState({});
  const [formData, setFormData] = useState({
    // ROS2 Code Generator
    ros2_urdf_path: '',
    ros2_target_controller: '',
    ros2_robot_joints: '',
    ros2_node_name: 'my_controller',
    ros2_additional_requirements: '',

    // Gazebo Scene Creator
    gazebo_robot_model_path: '',
    gazebo_environment_type: 'indoor',
    gazebo_objects: '',
    gazebo_lighting: 'default',

    // Quiz Generator
    quiz_topic: '',
    quiz_difficulty_level: 'intermediate',
    quiz_question_count: 5,
    quiz_question_types: 'multiple-choice',
    quiz_learning_objectives: ''
  });

  // Only show the panel to authenticated users
  if (!isAuthenticated) {
    return null;
  }

  const handleInputChange = (e, prefix = '') => {
    const { name, value } = e.target;
    const key = prefix ? `${prefix}_${name}` : name;
    setFormData(prev => ({
      ...prev,
      [key]: value
    }));
  };

  const handleArrayInputChange = (e, prefix = '') => {
    const { name, value } = e.target;
    const key = prefix ? `${prefix}_${name}` : name;
    // Convert comma-separated string to array
    const arrayValue = value.split(',').map(item => item.trim()).filter(item => item);
    setFormData(prev => ({
      ...prev,
      [key]: arrayValue
    }));
  };

  const callAgent = async (agentType) => {
    setLoading(true);
    setResults(prev => ({ ...prev, [agentType]: null }));

    try {
      let endpoint, payload;

      switch (agentType) {
        case 'ros2':
          endpoint = `${API_ENDPOINTS.QUERY}/agents/ros2-code-generator`; // This should be the correct endpoint
          payload = {
            urdf_path: formData.ros2_urdf_path,
            target_controller: formData.ros2_target_controller,
            robot_joints: formData.ros2_robot_joints || [],
            node_name: formData.ros2_node_name,
            additional_requirements: formData.ros2_additional_requirements
          };
          break;

        case 'gazebo':
          endpoint = `${API_ENDPOINTS.QUERY}/agents/gazebo-scene-creator`; // This should be the correct endpoint
          payload = {
            robot_model_path: formData.gazebo_robot_model_path,
            environment_type: formData.gazebo_environment_type,
            objects: formData.gazebo_objects || [],
            lighting: formData.gazebo_lighting
          };
          break;

        case 'quiz':
          endpoint = `${API_ENDPOINTS.QUERY}/agents/quiz-generator`; // This should be the correct endpoint
          payload = {
            topic: formData.quiz_topic,
            difficulty_level: formData.quiz_difficulty_level,
            question_count: parseInt(formData.quiz_question_count),
            question_types: [formData.quiz_question_types],
            learning_objectives: [formData.quiz_learning_objectives]
          };
          break;

        default:
          throw new Error('Invalid agent type');
      }

      // Use the correct API endpoint for agents - construct from base URL
      const baseUrl = API_ENDPOINTS.QUERY.replace('/api/v1/query', '');
      const response = await fetch(`${baseUrl}/api/v1/agents/${agentType}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload)
      });

      const result = await response.json();

      if (result.success) {
        setResults(prev => ({
          ...prev,
          [agentType]: result.result
        }));
      } else {
        setResults(prev => ({
          ...prev,
          [agentType]: { error: result.error || 'Unknown error occurred' }
        }));
      }
    } catch (error) {
      setResults(prev => ({
        ...prev,
        [agentType]: { error: error.message || 'Error calling agent' }
      }));
    } finally {
      setLoading(false);
    }
  };

  const renderROS2Form = () => (
    <div className="agent-form">
      <h3>ROS2 Code Generator</h3>
      <div className="form-group">
        <label htmlFor="ros2_urdf_path">URDF Path:</label>
        <input
          type="text"
          id="ros2_urdf_path"
          name="urdf_path"
          value={formData.ros2_urdf_path}
          onChange={(e) => handleInputChange(e, 'ros2')}
          placeholder="/path/to/robot.urdf"
        />
      </div>

      <div className="form-group">
        <label htmlFor="ros2_target_controller">Target Controller:</label>
        <input
          type="text"
          id="ros2_target_controller"
          name="target_controller"
          value={formData.ros2_target_controller}
          onChange={(e) => handleInputChange(e, 'ros2')}
          placeholder="position_controllers/JointGroupPositionController"
        />
      </div>

      <div className="form-group">
        <label htmlFor="ros2_robot_joints">Robot Joints (comma-separated):</label>
        <input
          type="text"
          id="ros2_robot_joints"
          name="robot_joints"
          value={formData.ros2_robot_joints}
          onChange={(e) => handleArrayInputChange(e, 'ros2')}
          placeholder="joint1, joint2, joint3"
        />
      </div>

      <div className="form-group">
        <label htmlFor="ros2_node_name">Node Name:</label>
        <input
          type="text"
          id="ros2_node_name"
          name="node_name"
          value={formData.ros2_node_name}
          onChange={(e) => handleInputChange(e, 'ros2')}
          placeholder="my_controller"
        />
      </div>

      <div className="form-group">
        <label htmlFor="ros2_additional_requirements">Additional Requirements:</label>
        <textarea
          id="ros2_additional_requirements"
          name="additional_requirements"
          value={formData.ros2_additional_requirements}
          onChange={(e) => handleInputChange(e, 'ros2')}
          placeholder="e.g., publish joint states, use PID control"
        />
      </div>

      <button
        className="button button--primary"
        onClick={() => callAgent('ros2')}
        disabled={loading}
      >
        {loading && activeTab === 'ros2' ? 'Generating...' : 'Generate ROS2 Code'}
      </button>
    </div>
  );

  const renderGazeboForm = () => (
    <div className="agent-form">
      <h3>Gazebo Scene Creator</h3>
      <div className="form-group">
        <label htmlFor="gazebo_robot_model_path">Robot Model Path:</label>
        <input
          type="text"
          id="gazebo_robot_model_path"
          name="robot_model_path"
          value={formData.gazebo_robot_model_path}
          onChange={(e) => handleInputChange(e, 'gazebo')}
          placeholder="/path/to/robot/model"
        />
      </div>

      <div className="form-group">
        <label htmlFor="gazebo_environment_type">Environment Type:</label>
        <select
          id="gazebo_environment_type"
          name="environment_type"
          value={formData.gazebo_environment_type}
          onChange={(e) => handleInputChange(e, 'gazebo')}
        >
          <option value="indoor">Indoor</option>
          <option value="outdoor">Outdoor</option>
          <option value="warehouse">Warehouse</option>
          <option value="maze">Maze</option>
          <option value="custom">Custom</option>
        </select>
      </div>

      <div className="form-group">
        <label htmlFor="gazebo_objects">Objects (comma-separated):</label>
        <input
          type="text"
          id="gazebo_objects"
          name="objects"
          value={formData.gazebo_objects}
          onChange={(e) => handleArrayInputChange(e, 'gazebo')}
          placeholder="box, cylinder, sphere"
        />
      </div>

      <div className="form-group">
        <label htmlFor="gazebo_lighting">Lighting:</label>
        <select
          id="gazebo_lighting"
          name="lighting"
          value={formData.gazebo_lighting}
          onChange={(e) => handleInputChange(e, 'gazebo')}
        >
          <option value="default">Default</option>
          <option value="bright">Bright</option>
          <option value="dim">Dim</option>
          <option value="night">Night</option>
        </select>
      </div>

      <button
        className="button button--primary"
        onClick={() => callAgent('gazebo')}
        disabled={loading}
      >
        {loading && activeTab === 'gazebo' ? 'Creating Scene...' : 'Create Gazebo Scene'}
      </button>
    </div>
  );

  const renderQuizForm = () => (
    <div className="agent-form">
      <h3>Quiz Generator</h3>
      <div className="form-group">
        <label htmlFor="quiz_topic">Topic:</label>
        <input
          type="text"
          id="quiz_topic"
          name="topic"
          value={formData.quiz_topic}
          onChange={(e) => handleInputChange(e, 'quiz')}
          placeholder="ROS2 Basics, Gazebo Simulation, etc."
        />
      </div>

      <div className="form-group">
        <label htmlFor="quiz_difficulty_level">Difficulty Level:</label>
        <select
          id="quiz_difficulty_level"
          name="difficulty_level"
          value={formData.quiz_difficulty_level}
          onChange={(e) => handleInputChange(e, 'quiz')}
        >
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
        </select>
      </div>

      <div className="form-group">
        <label htmlFor="quiz_question_count">Number of Questions:</label>
        <input
          type="number"
          id="quiz_question_count"
          name="question_count"
          value={formData.quiz_question_count}
          onChange={(e) => handleInputChange(e, 'quiz')}
          min="1"
          max="20"
        />
      </div>

      <div className="form-group">
        <label htmlFor="quiz_question_types">Question Type:</label>
        <select
          id="quiz_question_types"
          name="question_types"
          value={formData.quiz_question_types}
          onChange={(e) => handleInputChange(e, 'quiz')}
        >
          <option value="multiple-choice">Multiple Choice</option>
          <option value="true-false">True/False</option>
          <option value="short-answer">Short Answer</option>
          <option value="fill-in-blank">Fill in the Blank</option>
        </select>
      </div>

      <div className="form-group">
        <label htmlFor="quiz_learning_objectives">Learning Objectives:</label>
        <textarea
          id="quiz_learning_objectives"
          name="learning_objectives"
          value={formData.quiz_learning_objectives}
          onChange={(e) => handleInputChange(e, 'quiz')}
          placeholder="What should the learner know after taking this quiz?"
        />
      </div>

      <button
        className="button button--primary"
        onClick={() => callAgent('quiz')}
        disabled={loading}
      >
        {loading && activeTab === 'quiz' ? 'Generating Quiz...' : 'Generate Quiz'}
      </button>
    </div>
  );

  const renderResults = (agentType) => {
    const result = results[agentType];
    if (!result) return null;

    if (result.error) {
      return (
        <div className="agent-results error">
          <h4>Error:</h4>
          <p>{result.error}</p>
        </div>
      );
    }

    return (
      <div className="agent-results">
        <h4>Results:</h4>
        <pre className="results-content">
          {typeof result === 'string' ? result : JSON.stringify(result, null, 2)}
        </pre>
      </div>
    );
  };

  return (
    <div className="author-tools-panel">
      <h2>Author Tools Panel</h2>
      <p>Welcome, {user?.name || user?.email || 'author'}! Access reusable intelligence tools below.</p>

      <div className="tabs">
        <button
          className={`tab-button ${activeTab === 'ros2' ? 'active' : ''}`}
          onClick={() => setActiveTab('ros2')}
        >
          ROS2 Code Generator
        </button>
        <button
          className={`tab-button ${activeTab === 'gazebo' ? 'active' : ''}`}
          onClick={() => setActiveTab('gazebo')}
        >
          Gazebo Scene Creator
        </button>
        <button
          className={`tab-button ${activeTab === 'quiz' ? 'active' : ''}`}
          onClick={() => setActiveTab('quiz')}
        >
          Quiz Generator
        </button>
      </div>

      <div className="tab-content">
        {activeTab === 'ros2' && renderROS2Form()}
        {activeTab === 'gazebo' && renderGazeboForm()}
        {activeTab === 'quiz' && renderQuizForm()}
      </div>

      {results[activeTab] && renderResults(activeTab)}

      <style jsx>{`
        .author-tools-panel {
          margin: 2rem 0;
          padding: 1.5rem;
          border: 1px solid #ddd;
          border-radius: 8px;
          background-color: #f8f9fa;
        }

        .tabs {
          display: flex;
          margin-bottom: 1rem;
          border-bottom: 1px solid #ddd;
        }

        .tab-button {
          padding: 0.75rem 1.5rem;
          border: none;
          background: none;
          cursor: pointer;
          border-bottom: 3px solid transparent;
        }

        .tab-button.active {
          border-bottom-color: #25c2a0;
          font-weight: bold;
        }

        .tab-content {
          min-height: 300px;
        }

        .agent-form {
          padding: 1rem 0;
        }

        .form-group {
          margin-bottom: 1rem;
        }

        .form-group label {
          display: block;
          margin-bottom: 0.25rem;
          font-weight: bold;
        }

        .form-group input,
        .form-group select,
        .form-group textarea {
          width: 100%;
          padding: 0.5rem;
          border: 1px solid #ccc;
          border-radius: 4px;
          box-sizing: border-box;
        }

        .form-group textarea {
          min-height: 80px;
          resize: vertical;
        }

        .agent-results {
          margin-top: 1.5rem;
          padding: 1rem;
          background-color: white;
          border: 1px solid #ddd;
          border-radius: 4px;
          max-height: 400px;
          overflow: auto;
        }

        .agent-results.error {
          background-color: #ffecec;
          border-color: #e31e1e;
        }

        .results-content {
          white-space: pre-wrap;
          font-family: monospace;
          font-size: 0.9rem;
          margin: 0;
        }
      `}</style>
    </div>
  );
};

export default AuthorToolsPanel;