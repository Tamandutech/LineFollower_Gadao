<template>
  <div class="container">
    <div class="columns is-desktop is-centered">
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
                    id="file"
                    ref="file"
                    v-on:change="handleFileUpload()"
                  />
                  <span class="file-cta">
                    <upload-icon></upload-icon>
                    <span class="file-label">Selecionar</span>
                  </span>
                  <span class="file-name has-text-centered" v-html="fileName"></span>
                </label>
              </div>
              <br />
              <progress
                class="progress is-medium is-success"
                :value.prop="uploadPercentage"
                max="100"
              ></progress>
              <button class="button is-success is-outlined" v-on:click="submitFile()">Enviar</button>
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

export default {
  components: {
    UploadIcon
  },

  data() {
    return {
      file: "",
      fileName: "*.bin",
      uploadPercentage: 0
    };
  },

  methods: {
    /*
  Handles a change on the file upload
*/
    handleFileUpload() {
      this.file = this.$refs.file.files[0];
      this.fileName = event.target.files[0].name;
    },

    /* eslint-disable */
    /*
  Submits the file to the server
*/
    submitFile() {
      /*
    Initialize the form data
  */
      let formData = new FormData();

      /*
    Add the form data we need to submit
  */
      formData.append("file", this.file);

      /*
    Make the request to the POST /single-file URL
  */
      axios
        .post("http://192.168.15.20/update", formData, {
          headers: {
            "Content-Type": "multipart/form-data"
          },
          onUploadProgress: function(progressEvent) {
            this.uploadPercentage = parseInt(
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
