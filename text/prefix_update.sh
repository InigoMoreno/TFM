for name in bsc*
do
    newname=msc"$(echo "$name" | cut -c4-)"
    mv "$name" "$newname"
done
