import Vue from 'vue'
import Router from 'vue-router'

Vue.use(Router);

import Home from './views/Home.vue';
import About from './views/OTA.vue';
import Stats from './views/Stats.vue';


export default new Router({
  linkActiveClass: 'is-active',
  routes: [
    {
      path: '/',
      name: 'Inicio',
      component: Home
    }, 
    {
      path: '/ota',
      name: 'OTA',
      component: About
    },
    {
      path: '/info',
      name: 'Info',
      component: Stats
    }
  ]
})

