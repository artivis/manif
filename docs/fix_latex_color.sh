find site -type f -exec sed -i "s|svg.latex?|svg.latex?\\\color{white}|g" {} \;

#sed -i "s|png.latex?|png.latex?\\\color{white}|g" site/index.html
