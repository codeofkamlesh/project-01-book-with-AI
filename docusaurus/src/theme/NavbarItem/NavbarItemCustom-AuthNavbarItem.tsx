import React from 'react';
import AuthWidget from '@site/src/components/auth/AuthWidget';

// Docusaurus v3 custom navbar item component
const NavbarItemCustomAuthNavbarItem = () => {
  // This component is SSR-safe as it will only render the AuthWidget
  // which handles client-side logic internally
  return <AuthWidget />;
};

export default React.memo(NavbarItemCustomAuthNavbarItem);