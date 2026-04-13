$pdf_mode = 1;
$out_dir = 'build';
$aux_dir = 'build';
$max_repeat = 5;

$pdflatex = 'pdflatex %O -interaction=nonstopmode -file-line-error -synctex=1 %S';
$biber = 'biber --input-directory=build --output-directory=build %O %B';
$bibtex_use = 2;

$clean_ext = 'acn acr alg aux bbl bcf blg fdb_latexmk fls glg glo gls idx ilg ind lof log lot nav out run.xml snm synctex.gz toc xdv';
