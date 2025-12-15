import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';

// Layout wrapper to provide auth context to the entire app
const LayoutWrapper = ({ children }) => {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
};

export default LayoutWrapper;