<template>
  <div>
    <div class="grid">
      <div>
      <img id="stream" alt="Stream" src='https://192.168.0.1:81/stream'/>
      </div>
    </div>
    <div class="grid">
      <div></div>
    </div>
  </div>
</template>

<script>

export default {
  name: 'HomeView',
  methods:{
    goRight: function(){
      console.log("right");
    },
    goLeft: function(){
      console.log("left");
    },
    handleKeydown (e) {
      switch (e.keyCode) {
        case 37:
        this.goLeft();
        break;
        case 39: 
        this.goRight();
        break;
      }
    },
    handleKeyup (e) {
      console.log("keyup");
      console.log(e);
    }
  },
  beforeMount () {
    window.addEventListener('keydown', this.handleKeydown, null);
    window.addEventListener('keyup', this.handleKeyup, null);
  },
  beforeDestroy () {
    window.removeEventListener('keydown', this.handleKeydown);
    window.removeEventListener('keyup', this.handleKeyup);
  }
}
</script>

<style>

</style>