// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS2 Foundations',
      items: ['ros2-foundations/index'],
    },
    {
      type: 'category',
      label: 'Simulation',
      items: ['simulation/index'],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac',
      items: ['nvidia-isaac/index'],
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA)',
      items: ['vla-humanoids/index'],
    },
    {
      type: 'category',
      label: 'Cross-Model Integration',
      items: ['integration/index'],
    },
  ],
};

module.exports = sidebars;