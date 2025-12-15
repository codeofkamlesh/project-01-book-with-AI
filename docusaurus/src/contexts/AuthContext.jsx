import React, { createContext, useContext, useState, useEffect } from 'react';
import { API_ENDPOINTS } from '../utils/apiConfig';

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  // Check if user is logged in on component mount
  useEffect(() => {
    const checkAuthStatus = async () => {
      try {
        const response = await fetch(API_ENDPOINTS.AUTH_ME, {
          method: 'GET',
          credentials: 'include', // Include cookies for session
        });

        if (response.ok) {
          const userData = await response.json();
          setUser(userData);
        }
      } catch (error) {
        console.error('Auth check failed:', error);
      } finally {
        setLoading(false);
      }
    };

    checkAuthStatus();
  }, []);

  const login = async (email, password) => {
    try {
      // In a real implementation, this would call the Better-Auth login endpoint
      // For now, we'll simulate the login process
      const response = await fetch('/api/auth/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
        credentials: 'include', // Include cookies for session
      });

      if (response.ok) {
        const userData = await response.json();
        setUser(userData);
        return { success: true };
      } else {
        return { success: false, error: 'Invalid credentials' };
      }
    } catch (error) {
      console.error('Login error:', error);
      return { success: false, error: 'Login failed' };
    }
  };

  const signup = async (userData) => {
    try {
      // First, register with Better-Auth
      const authResponse = await fetch('/api/auth/register', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: userData.email,
          password: userData.password,
          name: userData.name,
        }),
        credentials: 'include', // Include cookies for session
      });

      if (authResponse.ok) {
        // After successful registration, get user info to get the user ID
        const userResponse = await fetch(API_ENDPOINTS.AUTH_ME, {
          method: 'GET',
          credentials: 'include', // Include cookies for session
        });

        if (userResponse.ok) {
          const userDataResponse = await userResponse.json();
          const userId = userDataResponse.user?.id || 'unknown';

          // Now save the extended profile with the actual user ID
          const profileResponse = await fetch(API_ENDPOINTS.AUTH_PROFILE, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              user_id: userId,
              software_background: {
                level: userData.softwareBackground,
                languages: userData.languagesUsed.split(',').map(lang => lang.trim()).filter(lang => lang)
              },
              hardware_background: {
                experience: userData.hardwareBackground,
                platforms: userData.rosExperience ? [userData.rosExperience] : []
              },
              preferences: {}
            }),
            credentials: 'include', // Include cookies for session
          });

          if (profileResponse.ok) {
            const profileData = await profileResponse.json();
            // After successful signup, set user as logged in
            setUser(userDataResponse.user);
            return { success: true };
          } else {
            // If profile save failed, try to logout the user
            await fetch('/api/auth/logout', {
              method: 'POST',
              credentials: 'include',
            });
            return { success: false, error: 'Failed to create profile' };
          }
        } else {
          return { success: false, error: 'Failed to get user info after registration' };
        }
      } else {
        const errorData = await authResponse.json();
        return { success: false, error: errorData.error || 'Registration failed' };
      }
    } catch (error) {
      console.error('Signup error:', error);
      return { success: false, error: 'Signup failed' };
    }
  };

  const logout = async () => {
    try {
      // In a real implementation, this would call the Better-Auth logout endpoint
      const response = await fetch('/api/auth/logout', {
        method: 'POST',
        credentials: 'include', // Include cookies for session
      });

      if (response.ok) {
        setUser(null);
      }
    } catch (error) {
      console.error('Logout error:', error);
    }
  };

  const value = {
    user,
    login,
    signup,
    logout,
    loading,
    isAuthenticated: !!user,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};