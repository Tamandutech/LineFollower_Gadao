<template>
  <!-- IRArray Card -->
  <div class="container">
    <div class="columns">
      <div class="column" v-for="(value, valueKey) in values">

        <div class="level-item has-text-centered">
          <div>
            <p class="heading">Sensor {{valueKey + 1}}</p>
            <p class="title">{{value}}</p>
          </div>
        </div>

        <!-- <span class="dot" :class="{'active': activity}"></span> -->
      </div>
    </div>
  </div>
</template>

<script>
export default {
  props: ["id", "name", "value"],

  computed: {
    values: function() {
      return this.value.split(",");
    }
  },

  data() {
    return {
      activity: true
    };
  },

  watch: {
    value: function() {
      this.activity = true;
      setTimeout(() => {
        this.activity = false;
      }, 100);
    }
  },

  mounted() {
    setTimeout(() => {
      this.activity = false;
    }, 500);
  }
};
</script>

<style scoped>
.card p {
  font-size: 4rem;
}
</style>