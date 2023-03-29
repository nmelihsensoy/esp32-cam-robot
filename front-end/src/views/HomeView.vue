<template>
  <div>
    <div class="grid">
      <div>
      <img id="stream" alt="Stream" src='http://192.168.0.1:81/stream'/>
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
      connection: null,
      lastKey: null
    }
  },
  methods:{
    goRight: function(){
      console.log("right");
      this.connection.send(JSON.stringify({"type" : "robot", "dir": 4}));
    },
    goLeft: function(){
      console.log("left");
      this.connection.send(JSON.stringify({"type" : "robot", "dir": 3}));
    },
    goForward: function(){
      console.log("forward");
      this.connection.send(JSON.stringify({"type" : "robot", "dir": 1}));
    },
    goBack: function(){
      console.log("back");
      this.connection.send(JSON.stringify({"type" : "robot", "dir": 2}));
    },
    stopRobot: function(){
      console.log("stop");
      this.connection.send(JSON.stringify({"type" : "robot", "dir": 0}));
    },
    handleKeydown (e) {

      if (e.keyCode == 37 ||
          e.keyCode == 38 ||
          e.keyCode == 39 ||
          e.keyCode == 40 ){
        e.preventDefault();
        e.stopPropagation();
      }

      if (this.lastKey == e.keyCode){
        return;
      }

      this.lastKey = e.keyCode
      switch (e.keyCode) {
        case 37:
        this.goLeft();
        break;
        case 39: 
        this.goRight();
        break;
        case 38: 
        this.goForward();
        break;
        case 40: 
        this.goBack();
        break;
      }
    },
    handleKeyup (e) {
      if(e.keyCode == 37 || 
      e.keyCode == 38 || 
      e.keyCode == 39 || 
      e.keyCode == 40){
        console.log("keyup_stop");
        //stop
        e.preventDefault();
        e.stopPropagation();
        this.lastKey = null;
        this.stopRobot();
      }
    }
  },
  created: function(){
    this.connection =  new WebSocket("ws://192.168.0.1:82/ws");
    this.connection.onmessage = function (event){
      console.log("MESSAGE");
      console.log(event);
    }
    this.connection.onopen = function(event){
      console.log("WS OPEN");
      console.log(event);
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