import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

const SignupPage = () => {
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
  const { signup, isAuthenticated } = useAuth();
  const history = useHistory();

  // If already authenticated, redirect to home
  if (isAuthenticated) {
    history.push('/');
  }

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    const result = await signup(formData);
    if (result.success) {
      history.push('/');
    } else {
      setError(result.error);
    }
  };

  return (
    <Layout title="Sign Up" description="Create your account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>Create Account</h2>
              </div>
              <div className="card__body">
                {error && (
                  <div className="alert alert--danger margin-bottom--md">
                    {error}
                  </div>
                )}

                <form onSubmit={handleSubmit}>
                  <div className="form-group margin-bottom--md">
                    <label htmlFor="name">Full Name</label>
                    <input
                      type="text"
                      id="name"
                      name="name"
                      className="form-control"
                      value={formData.name}
                      onChange={handleChange}
                      required
                    />
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="email">Email</label>
                    <input
                      type="email"
                      id="email"
                      name="email"
                      className="form-control"
                      value={formData.email}
                      onChange={handleChange}
                      required
                    />
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="password">Password</label>
                    <input
                      type="password"
                      id="password"
                      name="password"
                      className="form-control"
                      value={formData.password}
                      onChange={handleChange}
                      required
                    />
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="softwareBackground">Software Background</label>
                    <select
                      id="softwareBackground"
                      name="softwareBackground"
                      className="form-control"
                      value={formData.softwareBackground}
                      onChange={handleChange}
                    >
                      <option value="none">None</option>
                      <option value="beginner">Beginner</option>
                      <option value="intermediate">Intermediate</option>
                      <option value="advanced">Advanced</option>
                    </select>
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="languagesUsed">Programming Languages Used</label>
                    <input
                      type="text"
                      id="languagesUsed"
                      name="languagesUsed"
                      className="form-control"
                      placeholder="e.g., Python, C++, JavaScript"
                      value={formData.languagesUsed}
                      onChange={handleChange}
                    />
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="hardwareBackground">Hardware Background</label>
                    <select
                      id="hardwareBackground"
                      name="hardwareBackground"
                      className="form-control"
                      value={formData.hardwareBackground}
                      onChange={handleChange}
                    >
                      <option value="none">None</option>
                      <option value="basic robotics">Basic Robotics</option>
                      <option value="jetson/embedded">Jetson/Embedded</option>
                      <option value="ros experience">ROS Experience</option>
                    </select>
                  </div>

                  <div className="form-group margin-bottom--lg">
                    <label htmlFor="rosExperience">ROS Experience Level</label>
                    <select
                      id="rosExperience"
                      name="rosExperience"
                      className="form-control"
                      value={formData.rosExperience}
                      onChange={handleChange}
                    >
                      <option value="">Select your ROS experience</option>
                      <option value="no experience">No Experience</option>
                      <option value="ros1">ROS 1</option>
                      <option value="ros2">ROS 2</option>
                      <option value="ros2 + navigation">ROS 2 + Navigation</option>
                      <option value="ros2 + perception">ROS 2 + Perception</option>
                    </select>
                  </div>

                  <button type="submit" className="button button--primary button--block">
                    Sign Up
                  </button>
                </form>
              </div>

              <div className="card__footer">
                <p>
                  Already have an account?{' '}
                  <a href="/login">Sign in here</a>
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default SignupPage;