import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

const LoginPage = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { login, isAuthenticated } = useAuth();
  const history = useHistory();

  // If already authenticated, redirect to home
  if (isAuthenticated) {
    history.push('/');
  }

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    const result = await login(email, password);
    if (result.success) {
      history.push('/');
    } else {
      setError(result.error);
    }
  };

  return (
    <Layout title="Sign In" description="Sign in to your account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>Sign In</h2>
              </div>
              <div className="card__body">
                {error && (
                  <div className="alert alert--danger margin-bottom--md">
                    {error}
                  </div>
                )}

                <form onSubmit={handleSubmit}>
                  <div className="form-group margin-bottom--md">
                    <label htmlFor="email">Email</label>
                    <input
                      type="email"
                      id="email"
                      className="form-control"
                      value={email}
                      onChange={(e) => setEmail(e.target.value)}
                      required
                    />
                  </div>

                  <div className="form-group margin-bottom--lg">
                    <label htmlFor="password">Password</label>
                    <input
                      type="password"
                      id="password"
                      className="form-control"
                      value={password}
                      onChange={(e) => setPassword(e.target.value)}
                      required
                    />
                  </div>

                  <button type="submit" className="button button--primary button--block">
                    Sign In
                  </button>
                </form>
              </div>

              <div className="card__footer">
                <p>
                  Don't have an account?{' '}
                  <a href="/signup">Sign up here</a>
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default LoginPage;