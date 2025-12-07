import React from 'react';
import clsx from 'clsx';

export default function Admonition({ type = "note", title, children, className }) {
  const typeClasses = {
    note: "alert alert--info",
    tip: "alert alert--success",
    caution: "alert alert--warning",
    warning: "alert alert--danger",
    secondary: "alert alert--secondary"
  };

  const defaultTitles = {
    note: "Note",
    tip: "Tip",
    caution: "Caution",
    warning: "Warning",
    secondary: "Secondary"
  };

  const displayTitle = title || defaultTitles[type] || defaultTitles.note;

  return (
    <div className={clsx(typeClasses[type] || typeClasses.note, className)}>
      <div className="admonition-heading">
        <h5>{displayTitle}</h5>
      </div>
      <div className="admonition-content">
        {children}
      </div>
    </div>
  );
}