import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import { useLocation } from '@docusaurus/router';

export default function TextbookLayout(props) {
  const { children, title, description, toc, sidebar } = props;
  const location = useLocation();

  // Add special styling for textbook pages
  const isTextbookPage = location.pathname.includes('/docs/');

  return (
    <Layout title={title} description={description}>
      <div className="container margin-vert--lg textbook-content">
        <div className="row">
          <div className={clsx('col', {
            'col--8': isTextbookPage && toc,
            'col--9': isTextbookPage && !toc,
            'col--12': !isTextbookPage
          })}>
            <main>
              {children}
            </main>
          </div>

          {toc && (
            <div className="col col--4">
              <div className="table-of-contents textbook-toc">
                <h3>Chapter Contents</h3>
                {toc}
              </div>
            </div>
          )}
        </div>
      </div>
    </Layout>
  );
}