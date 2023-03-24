<template>
  <div>
    <div class="grid">
      <div>
      <img id="stream" alt="Stream" src='http://192.168.0.1:81/stream' style="object-fit: contain; height: auto; width:100%; max-height: 60%;"/>
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
  data: function(){
    return{
      connection: null
    }
  },
  methods:{
    sendSocketMsg: function(msg){
      this.connection.send(msg);
    },
    goRight: function(){
      console.log("right");
      this.sendSocketMsg('{"type":"robot", "cmd": "right"}');
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
        case 40:
        e.preventDefault();
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
  },
  created: function(){
    this.connection = new WebSocket('ws://192.168.0.1:82/ws');

    this.connection.onmessage = function(event){
      console.log(event);
    }

    this.connection.onopen = function(event){
      console.log(event);
      console.log("socket connected");
    }

  }
}
</script>

<style>

</style>