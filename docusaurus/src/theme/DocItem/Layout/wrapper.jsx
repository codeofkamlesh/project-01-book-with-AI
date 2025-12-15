import React from 'react';
import { useDoc } from '@docusaurus/theme-common/internal';
import { useLocation } from '@docusaurus/router';
import PersonalizeButton from '@site/src/components/personalize/PersonalizeButton';

// Wrapper for DocItem layout to inject PersonalizeButton at the top of chapter pages
const DocItemLayoutWrapper = ({ children }) => {
  const { metadata } = useDoc();
  const location = useLocation();

  // Only show PersonalizeButton on docs pages, not on homepage or other pages
  const isDocsPage = location.pathname.startsWith('/docs/');

  return (
    <>
      {isDocsPage && (
        <div className="personalize-section">
          <PersonalizeButton docPath={metadata.source} />
        </div>
      )}
      {children}
      <style jsx>{`
        .personalize-section {
          margin-bottom: 1.5rem;
        }
      `}</style>
    </>
  );
};

export default DocItemLayoutWrapper;