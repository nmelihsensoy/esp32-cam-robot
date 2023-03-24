<template>
  <div id="app">
    <main class="container-fluid">
      <nav>
        <ul>
          <li><router-link tag="a" to="/">Home</router-link></li>
        </ul>
        <ul>
          <li><strong>Robot Control</strong></li>
        </ul>
        <ul>
          <li><router-link tag="a" to="/about">Settings</router-link></li>
        </ul>
      </nav>
      <router-view/>
    </main>
  </div>
</template>

<style>

</style>
