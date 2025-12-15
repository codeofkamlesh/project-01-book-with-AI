import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import AuthorToolsPanel from '../components/agents/AuthorToolsPanel';

function AuthorToolsPage() {
  const { user, isAuthenticated, loading } = useAuth();

  if (loading) {
    return (
      <Layout title="Author Tools" description="Author tools panel for reusable intelligence">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <h1>Loading...</h1>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return (
      <Layout title="Author Tools" description="Author tools panel for reusable intelligence">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <h1>Access Denied</h1>
              <p>Please sign in to access the author tools panel.</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Author Tools Panel" description="Reusable intelligence tools for content authors">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <AuthorToolsPanel />
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default AuthorToolsPage;