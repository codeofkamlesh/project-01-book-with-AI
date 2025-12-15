import React from 'react';
import AuthComponent from '../../components/auth/AuthComponent';

// Custom navbar item component for authentication
const CustomAuthComponent = (props) => {
  return (
    <div style={{ display: 'flex', alignItems: 'center', height: '100%' }}>
      <AuthComponent {...props} />
    </div>
  );
};

export default CustomAuthComponent;