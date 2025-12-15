// Lightweight client-side script to handle auth status display
document.addEventListener('DOMContentLoaded', function() {
  const authStatusElement = document.getElementById('auth-status');

  if (authStatusElement) {
    // Fetch current auth status
    fetch('/api/v1/me') // Using the existing endpoint
      .then(response => response.json())
      .then(data => {
        if (data.user && data.user.email) {
          // User is logged in
          authStatusElement.innerHTML = `Logout (${data.user.email})`;
          authStatusElement.style.cursor = 'pointer';

          // Add click handler for logout
          authStatusElement.addEventListener('click', function() {
            if (confirm('Are you sure you want to logout?')) {
              // For now, redirect to logout endpoint (this would be implemented in the backend)
              window.location.href = '/logout'; // This will need backend implementation
            }
          });
        } else {
          // User is not logged in
          authStatusElement.innerHTML = '<a href="/login">Sign In</a>';
        }
      })
      .catch(error => {
        console.error('Error fetching auth status:', error);
        // On error, show sign in link
        authStatusElement.innerHTML = '<a href="/login">Sign In</a>';
      });
  }
});