import Vue from 'vue'
import VueApexCharts from 'vue-apexcharts';
import axios from 'axios'
import VueAxios from 'vue-axios'

Vue.use(VueApexCharts, VueAxios, axios)
Vue.component('apexchart', VueApexCharts)

import './styles/custom.scss';

import App from './App.vue'
import router from './router'

Vue.config.productionTip = false

router.beforeEach((to, from, next) => {
  let title = to.name;
  document.title = title+" - Tamandutech";
  next();
});

new Vue({
  router,
  render: h => h(App)
}).$mount('#app')
