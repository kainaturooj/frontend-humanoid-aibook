import React from 'react';
import clsx from 'clsx';

export default function LearningObjectives({ objectives, className }) {
  return (
    <div className={clsx('learning-objectives', className)}>
      <h4>Learning Objectives</h4>
      <ul>
        {objectives.map((objective, index) => (
          <li key={index}>{objective}</li>
        ))}
      </ul>
    </div>
  );
}