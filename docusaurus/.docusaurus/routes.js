import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/project-01-book-with-AI/docs',
    component: ComponentCreator('/project-01-book-with-AI/docs', 'd31'),
    routes: [
      {
        path: '/project-01-book-with-AI/docs',
        component: ComponentCreator('/project-01-book-with-AI/docs', '058'),
        routes: [
          {
            path: '/project-01-book-with-AI/docs',
            component: ComponentCreator('/project-01-book-with-AI/docs', '935'),
            routes: [
              {
                path: '/project-01-book-with-AI/docs/integration/',
                component: ComponentCreator('/project-01-book-with-AI/docs/integration/', 'd13'),
                exact: true
              },
              {
                path: '/project-01-book-with-AI/docs/nvidia-isaac/',
                component: ComponentCreator('/project-01-book-with-AI/docs/nvidia-isaac/', 'fba'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/project-01-book-with-AI/docs/ros2-foundations/',
                component: ComponentCreator('/project-01-book-with-AI/docs/ros2-foundations/', '1b5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/project-01-book-with-AI/docs/simulation/',
                component: ComponentCreator('/project-01-book-with-AI/docs/simulation/', '1eb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/project-01-book-with-AI/docs/vla-humanoids/',
                component: ComponentCreator('/project-01-book-with-AI/docs/vla-humanoids/', '925'),
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
    path: '/project-01-book-with-AI/',
    component: ComponentCreator('/project-01-book-with-AI/', '043'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
