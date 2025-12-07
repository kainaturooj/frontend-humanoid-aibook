import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug', '109'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/config', '3c1'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/content', '783'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/globalData', '58e'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/metadata', '523'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/registry', 'c50'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/routes', '534'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/docs',
    component: ComponentCreator('/physical-ai-humanoid-robotics/docs', 'bb3'),
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
    component: ComponentCreator('/physical-ai-humanoid-robotics/', '221'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
