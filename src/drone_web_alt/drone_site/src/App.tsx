import type { Component } from 'solid-js';

import logo from './logo.svg';
import styles from './App.module.css';
import { Events } from './Events';

const App: Component = () => {
  return (
    <div class={styles.App}>
      <Events />
    </div>
  );
};

export default App;
