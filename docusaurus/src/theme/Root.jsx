import React from 'react';
import ChatWidget from '../components/rag/ChatWidget';
import { AuthProvider } from '../contexts/AuthContext';

// Root component that wraps the entire Docusaurus application
// This ensures the Auth context is available on every page
// ChatWidget temporarily disabled for runtime stability
export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      {/* ChatWidget temporarily disabled for runtime safety */}
      {/* <ChatWidget /> */}
    </AuthProvider>
  );
}