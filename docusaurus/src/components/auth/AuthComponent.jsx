import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';

const AuthComponent = () => {
  const { user, login, signup, logout, isAuthenticated } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [isLoginView, setIsLoginView] = useState(true);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    softwareBackground: 'beginner',
    hardwareBackground: 'none',
    languagesUsed: '',
    rosExperience: ''
  });
  const [error, setError] = useState('');

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleLogin = async (e) => {
    e.preventDefault();
    setError('');
    const result = await login(formData.email, formData.password);
    if (!result.success) {
      setError(result.error);
    } else {
      setShowAuthModal(false);
    }
  };

  const handleSignup = async (e) => {
    e.preventDefault();
    setError('');
    const result = await signup(formData);
    if (!result.success) {
      setError(result.error);
    } else {
      setShowAuthModal(false);
    }
  };

  const toggleView = () => {
    setIsLoginView(!isLoginView);
    setError('');
  };

  if (isAuthenticated && user) {
    return (
      <div className="auth-component">
        <div className="dropdown dropdown--right dropdown--nocaret">
          <button className="navbar__link dropdown__trigger">
            {user.name || user.email}
          </button>
          <ul className="dropdown__menu">
            <li>
              <a
                className="dropdown__item"
                href="/author-tools"
              >
                Author Tools
              </a>
            </li>
            <li>
              <button
                className="dropdown__item"
                onClick={logout}
              >
                Sign Out
              </button>
            </li>
          </ul>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-component">
      <button
        className="button button--secondary button--sm"
        onClick={() => setShowAuthModal(true)}
      >
        Sign In
      </button>

      {showAuthModal && (
        <div className="auth-modal">
          <div className="auth-modal-overlay" onClick={() => setShowAuthModal(false)}></div>
          <div className="auth-modal-content">
            <div className="auth-modal-header">
              <h3>{isLoginView ? 'Sign In' : 'Sign Up'}</h3>
              <button
                className="auth-modal-close"
                onClick={() => setShowAuthModal(false)}
              >
                ×
              </button>
            </div>

            {error && <div className="auth-error">{error}</div>}

            <form onSubmit={isLoginView ? handleLogin : handleSignup}>
              {!isLoginView && (
                <div className="form-group">
                  <label htmlFor="name">Full Name</label>
                  <input
                    type="text"
                    id="name"
                    name="name"
                    value={formData.name}
                    onChange={handleInputChange}
                    required={!isLoginView}
                  />
                </div>
              )}

              <div className="form-group">
                <label htmlFor="email">Email</label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  value={formData.email}
                  onChange={handleInputChange}
                  required
                />
              </div>

              <div className="form-group">
                <label htmlFor="password">Password</label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  value={formData.password}
                  onChange={handleInputChange}
                  required
                />
              </div>

              {!isLoginView && (
                <>
                  <div className="form-group">
                    <label htmlFor="softwareBackground">Software Background</label>
                    <select
                      id="softwareBackground"
                      name="softwareBackground"
                      value={formData.softwareBackground}
                      onChange={handleInputChange}
                    >
                      <option value="none">None</option>
                      <option value="beginner">Beginner</option>
                      <option value="intermediate">Intermediate</option>
                      <option value="advanced">Advanced</option>
                    </select>
                  </div>

                  <div className="form-group">
                    <label htmlFor="languagesUsed">Programming Languages Used</label>
                    <input
                      type="text"
                      id="languagesUsed"
                      name="languagesUsed"
                      placeholder="e.g., Python, C++, JavaScript"
                      value={formData.languagesUsed}
                      onChange={handleInputChange}
                    />
                  </div>

                  <div className="form-group">
                    <label htmlFor="hardwareBackground">Hardware Background</label>
                    <select
                      id="hardwareBackground"
                      name="hardwareBackground"
                      value={formData.hardwareBackground}
                      onChange={handleInputChange}
                    >
                      <option value="none">None</option>
                      <option value="basic robotics">Basic Robotics</option>
                      <option value="jetson/embedded">Jetson/Embedded</option>
                      <option value="ros experience">ROS Experience</option>
                    </select>
                  </div>

                  <div className="form-group">
                    <label htmlFor="rosExperience">ROS Experience Level</label>
                    <select
                      id="rosExperience"
                      name="rosExperience"
                      value={formData.rosExperience}
                      onChange={handleInputChange}
                    >
                      <option value="">Select your ROS experience</option>
                      <option value="no experience">No Experience</option>
                      <option value="ros1">ROS 1</option>
                      <option value="ros2">ROS 2</option>
                      <option value="ros2 + navigation">ROS 2 + Navigation</option>
                      <option value="ros2 + perception">ROS 2 + Perception</option>
                    </select>
                  </div>
                </>
              )}

              <button type="submit" className="button button--primary">
                {isLoginView ? 'Sign In' : 'Sign Up'}
              </button>
            </form>

            <div className="auth-toggle">
              <button onClick={toggleView}>
                {isLoginView
                  ? "Don't have an account? Sign up"
                  : "Already have an account? Sign in"}
              </button>
            </div>
          </div>

          <style jsx>{`
            .auth-modal {
              position: fixed;
              top: 0;
              left: 0;
              right: 0;
              bottom: 0;
              display: flex;
              align-items: center;
              justify-content: center;
              z-index: 1000;
            }

            .auth-modal-overlay {
              position: absolute;
              top: 0;
              left: 0;
              right: 0;
              bottom: 0;
              background-color: rgba(0, 0, 0, 0.5);
            }

            .auth-modal-content {
              position: relative;
              background: white;
              padding: 1.5rem;
              border-radius: 8px;
              box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
              width: 90%;
              max-width: 400px;
              z-index: 1001;
            }

            .auth-modal-header {
              display: flex;
              justify-content: space-between;
              align-items: center;
              margin-bottom: 1rem;
            }

            .auth-modal-close {
              background: none;
              border: none;
              font-size: 1.5rem;
              cursor: pointer;
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
            .form-group select {
              width: 100%;
              padding: 0.5rem;
              border: 1px solid #ccc;
              border-radius: 4px;
              box-sizing: border-box;
            }

            .auth-error {
              color: #e31e1e;
              margin-bottom: 1rem;
              padding: 0.5rem;
              background-color: #ffecec;
              border-radius: 4px;
            }

            .auth-toggle {
              margin-top: 1rem;
              text-align: center;
            }

            .auth-toggle button {
              background: none;
              border: none;
              color: #25c2a0;
              cursor: pointer;
              text-decoration: underline;
            }
          `}</style>
        </div>
      )}
    </div>
  );
};

export default AuthComponent;