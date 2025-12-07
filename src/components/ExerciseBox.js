import React from 'react';
import clsx from 'clsx';

export default function ExerciseBox({ title, children, type = "exercise", className }) {
  const boxTitle = type === "assessment" ? "Assessment" : type === "example" ? "Example" : "Exercise";

  return (
    <div className={clsx('exercise-box', className)}>
      <h3>{boxTitle}</h3>
      {children}
    </div>
  );
}