const fs = require('fs');

module.exports = (dir, callback) => {
  // Also check out the path module for help with
  // sanitizing and normalizing paths!
  const extension = 'data';
  const fileList = [];

  fs.readdir(dir, (err, list) => {
    if (err) {
      callback(err);
    } else {
      // console.log(list);
      list.forEach((file) => {
        const splitFile = file.split('.');
        // This also lists files named the extension,
        // So we check that the filename HAS an extension first
        if (splitFile.length > 1) {
          if (splitFile[splitFile.length - 1] === extension) {
            fileList.push(file);
          }
        }
      });
      callback(null, fileList);
    }
  });

  // Official solution:
  // var fs = require('fs')
  // var path = require('path')
  //
  // fs.readdir(process.argv[2], function (err, list) {
  //     list.forEach(function (file) {
  //         if (path.extname(file) === '.' + process.argv[3])
  //             console.log(file)
  //         })
  //     })
};
