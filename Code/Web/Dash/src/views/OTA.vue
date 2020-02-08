<template>
  <div class="container">
    <div class="columns is-desktop is-centered">
      <!-- Atualização OTA -->
      <div class="column is-half">
        <div class="box is" style="width: 100%;">
          <div class="has-text-centered">
            <p class="title is-5">Atualização OTA</p>
            <p class="subtitle">Envie o arquivo .bin gerado pelo compilador.</p>
            <div class="large-12 medium-12 small-12 cell">
              <div class="file has-name is-boxed is-centered">
                <label class="file-label">
                  <input
                    class="file-input"
                    type="file"
                    id="fileOTA"
                    ref="fileOTA"
                    v-on:change="handleFileUploadOTA()"
                  />
                  <span class="file-cta">
                    <upload-icon></upload-icon>
                    <span class="file-label">Selecionar</span>
                  </span>
                  <span class="file-name has-text-centered" v-html="fileNameOTA"></span>
                </label>
              </div>
              <br />
              <progress
                class="progress is-medium is-success"
                :value.prop="uploadPercentageOTA"
                max="100"
              ></progress>
              <button class="button is-success is-outlined" v-on:click="submitFileOTA()">Enviar</button>
            </div>
          </div>
        </div>
      </div>

      <div class="modal" v-bind:class="{'is-active':this.isActiveModal}">
        <div class="modal-background"></div>
        <div class="modal-card">
          <header class="modal-card-head">
            <p class="modal-card-title">{{tituloModal}}</p>
            <button @click="closeModal" class="delete" aria-label="close"></button>
          </header>
          <section class="modal-card-body">{{messageModal}}</section>
          <footer class="modal-card-foot">
            <button class="button is-success">Save changes</button>
            <button class="button">Cancel</button>
          </footer>
        </div>
      </div>

      <!-- Atualização SPIFFS -->
      <div class="column is-half">
        <div class="box is" style="width: 100%;">
          <div class="has-text-centered">
            <p class="title is-5">Atualização SPIFFS</p>
            <p class="subtitle">Envie o arquivo .html.gz gerado pelo NPM.</p>
            <div class="large-12 medium-12 small-12 cell">
              <div class="file has-name is-boxed is-centered">
                <label class="file-label">
                  <input
                    class="file-input"
                    type="file"
                    id="fileSPIFFS"
                    ref="fileSPIFFS"
                    v-on:change="handleFileUploadSPIFFS()"
                  />
                  <span class="file-cta">
                    <upload-icon></upload-icon>
                    <span class="file-label">Selecionar</span>
                  </span>
                  <span class="file-name has-text-centered" v-html="fileNameSPIFFS"></span>
                </label>
              </div>
              <br />
              <progress
                class="progress is-medium is-success"
                :value.prop="uploadPercentageSPIFFS"
                max="100"
              ></progress>
              <button class="button is-success is-outlined" v-on:click="submitFileSPIFFS()">Enviar</button>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>
  
<script>
import { UploadIcon } from "vue-feather-icons";
import axios from "axios";

//let host = document.location.host;
let host = "192.168.15.167";

export default {
  components: {
    UploadIcon
  },

  data() {
    return {
      fileOTA: "",
      fileNameOTA: "*.bin",
      uploadPercentageOTA: 0,
      fileSPIFFS: "",
      fileNameSPIFFS: "*.html.gz",
      uploadPercentageSPIFFS: 0,
      isActiveModal: false,
      tituloModal: "",
      messageModal: ""
    };
  },

  methods: {

    closeModal: function() {
      this.isActiveModal = false;
    },

    /*
  Handles a change on the file upload
*/
    handleFileUploadOTA() {
      this.fileOTA = this.$refs.fileOTA.files[0];
      this.fileNameOTA = event.target.files[0].name;
    },

    handleFileUploadSPIFFS() {
      this.fileSPIFFS = this.$refs.fileSPIFFS.files[0];
      this.fileNameSPIFFS = event.target.files[0].name;
    },

    /* eslint-disable */
    /*
  Submits the file to the server
*/
    submitFileOTA() {
      /*
    Initialize the form data
  */
      let formDataOTA = new FormData();

      /*
    Add the form data we need to submit
  */
      formDataOTA.append("fileOTA", this.fileOTA);

      /*
    Make the request to the POST /single-file URL
  */
      axios
        .post("http://" + host + "/update", formDataOTA, {
          headers: {
            "Content-Type": "multipart/form-data"
          },
          onUploadProgress: function(progressEvent) {
            this.uploadPercentageOTA = parseInt(
              Math.round((progressEvent.loaded / progressEvent.total) * 100)
            );
          }.bind(this)
        })
        .then(function() {
          console.log("SUCCESS!!");
        },)
        .then(() => {
          this.tituloModal = "Sucesso!!! 😄"
          this.messageModal = "Upload do arquivo concluído!";
          this.isActiveModal = true;
          this.uploadPercentageOTA = 0;
        })
        .catch(function() {
          console.log("FAILURE!!");
        })
        .catch(() => {
          this.tituloModal = "Falha... 😢"
          this.messageModal = "O upload do arquivo falhou, tente novamente...";
          this.isActiveModal = true;
          this.uploadPercentageOTA = 0;
        });
    },

    submitFileSPIFFS() {
      /*
    Initialize the form data
  */
      let formDataSPIFFS = new FormData();

      /*
    Add the form data we need to submit
  */
      formDataSPIFFS.append("fileSPIFFS", this.fileSPIFFS);

      /*
    Make the request to the POST /single-file URL
  */
      axios
        .post("http://" + host + "/updatespiffs", formDataSPIFFS, {
          headers: {
            "Content-Type": "multipart/form-data"
          },
          onUploadProgress: function(progressEvent) {
            this.uploadPercentageSPIFFS = parseInt(
              Math.round((progressEvent.loaded / progressEvent.total) * 100)
            );
          }.bind(this)
        })
        .then(function() {
          console.log("SUCCESS!!");
        })
        .catch(function() {
          console.log("FAILURE!!");
        });
    }
  }
};
</script>
