// API Configuration for Physical AI & Humanoid Robotics Book

// Get the API base URL from global window variable or default to relative path
const getApiBaseUrl = () => {
  // Check if we're in a browser environment
  if (typeof window !== 'undefined' && window.__API_BASE_URL__) {
    return window.__API_BASE_URL__;
  }

  // Fallback to default API path
  return '/api/v1';
};

export const API_BASE_URL = getApiBaseUrl();

// API endpoints
export const API_ENDPOINTS = {
  QUERY: `${API_BASE_URL}/query`,
  INGEST: `${API_BASE_URL}/ingest/docs`,
  REINDEX: `${API_BASE_URL}/reindex`,
  SESSION_CONTEXT: `${API_BASE_URL}/session/answer-context`,
  AUTH_PROFILE: `${API_BASE_URL}/auth/profile`,
  AUTH_ME: `${API_BASE_URL}/auth/me`,
  AUTH_CALLBACK: `${API_BASE_URL}/auth/better-auth-callback`,
  PERSONALIZE_RENDER: `${API_BASE_URL}/personalize/render`,
  TRANSLATE_URDU: `${API_BASE_URL}/translate/urdu`,
  TRANSLATE: `${API_BASE_URL}/translate`,
  STATUS: `${API_BASE_URL}/status`,
};

export default API_ENDPOINTS;