import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/author-tools',
    component: ComponentCreator('/author-tools', '2c8'),
    exact: true
  },
  {
    path: '/login',
    component: ComponentCreator('/login', 'f43'),
    exact: true
  },
  {
    path: '/signup',
    component: ComponentCreator('/signup', '312'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'ede'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'd9c'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '5de'),
            routes: [
              {
                path: '/docs/integration/',
                component: ComponentCreator('/docs/integration/', '5dc'),
                exact: true
              },
              {
                path: '/docs/nvidia-isaac/',
                component: ComponentCreator('/docs/nvidia-isaac/', '5e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-foundations/',
                component: ComponentCreator('/docs/ros2-foundations/', '09c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/',
                component: ComponentCreator('/docs/simulation/', '992'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla-humanoids/',
                component: ComponentCreator('/docs/vla-humanoids/', 'da6'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
