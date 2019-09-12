// Finalize Nodejs Script
// 1 - Append JS in HTML Document
// 2 - Gzip HTML
// 3 - Covert to Raw Bytes
// 4 - ( Save to File: webpage.h ) in dist Folder

const fs = require('fs');
const zlib = require('zlib');
const { gzip } = require('@gfx/zopfli');

function getByteArray(file){
    let fileData = file.toString('hex');
    let result = [];
    for (let i = 0; i < fileData.length; i+=2)
      result.push('0x'+fileData[i]+''+fileData[i+1]);
    return result;
}

let html = fs.readFileSync(__dirname+'/web/index.html');
// Gzip the index.html file with JS Code.
const gzippedIndex = zlib.gzipSync(html, {'level': zlib.constants.Z_BEST_COMPRESSION});
let indexHTML = getByteArray(gzippedIndex);

let source = 
`
const uint32_t ELEGANT_HTML_SIZE = ${indexHTML.length};
const uint8_t ELEGANT_HTML[] PROGMEM = { ${indexHTML} };
`;


fs.writeFileSync(__dirname+'/web/elegantWebpage.h', source, 'utf8');

// Produce a second variant with zopfli
// Zopfli is a improved zip algorithm by google
// Takes much more time and maybe is not available on every machine
const input =  html;
gzip(input, {numiterations: 15}, (err, output) => {
    indexHTML = output;
    let source =
`
const uint32_t ELEGANT_HTML_SIZE = ${indexHTML.length};
const uint8_t ELEGANT_HTML[] PROGMEM = { ${indexHTML} };
`;

    fs.writeFileSync(__dirname + '/web/webpage_zopfli.h', source, 'utf8');
});
