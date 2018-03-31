import React from 'react';
import ReactDOM from 'react-dom';
import 'bootstrap/dist/css/bootstrap.css';
import './index.css';
import App from './containers/App';
// This doesn't work without HTTPS,
// and HTTPS won't work for a non-routable address without
// accepting self-signed certificates in your browser.
// import registerServiceWorker from './registerServiceWorker';

ReactDOM.render(<App />, document.getElementById('root'));
// registerServiceWorker();
