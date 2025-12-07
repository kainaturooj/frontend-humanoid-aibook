import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug', '62b'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/config', '239'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/content', '1e3'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/globalData', '9ff'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/metadata', 'aee'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/registry', '0b2'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/routes', '7fe'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/docs',
    component: ComponentCreator('/physical-ai-humanoid-robotics/docs', 'aec'),
    routes: [
      {
        path: '/physical-ai-humanoid-robotics/docs',
        component: ComponentCreator('/physical-ai-humanoid-robotics/docs', '0bb'),
        routes: [
          {
            path: '/physical-ai-humanoid-robotics/docs',
            component: ComponentCreator('/physical-ai-humanoid-robotics/docs', '5eb'),
            routes: [
              {
                path: '/physical-ai-humanoid-robotics/docs/intro',
                component: ComponentCreator('/physical-ai-humanoid-robotics/docs/intro', '3d2'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/docs/ros2-basics/python-ros2-integration',
                component: ComponentCreator('/physical-ai-humanoid-robotics/docs/ros2-basics/python-ros2-integration', '6c1'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/docs/ros2-basics/ros2-fundamentals',
                component: ComponentCreator('/physical-ai-humanoid-robotics/docs/ros2-basics/ros2-fundamentals', '90e'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/docs/ros2-basics/urdf-essentials',
                component: ComponentCreator('/physical-ai-humanoid-robotics/docs/ros2-basics/urdf-essentials', '592'),
                exact: true,
                sidebar: "textbookSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/physical-ai-humanoid-robotics/',
    component: ComponentCreator('/physical-ai-humanoid-robotics/', 'aa5'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
