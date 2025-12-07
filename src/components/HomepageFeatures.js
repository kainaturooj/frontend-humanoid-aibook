import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Educational Excellence',
    description: (
      <>
        Comprehensive coverage of Physical AI & Humanoid Robotics with clear concepts,
        practical examples, and structured learning paths.
      </>
    ),
  },
  {
    title: 'Technical Accuracy',
    description: (
      <>
        Content aligned with official ROS 2, Gazebo, Unity, and Isaac documentation
        for maximum technical accuracy.
      </>
    ),
  },
  {
    title: 'Interactive Learning',
    description: (
      <>
        Integrated RAG chatbot provides personalized assistance grounded in textbook content.
      </>
    ),
  },
];

function Feature({ Svg, title, description }) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}