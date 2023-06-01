import { Component, createResource } from 'solid-js';

import logo from './logo.png';
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
