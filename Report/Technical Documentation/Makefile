# zonder extensie
ROOT = technical_document
BIB = biblio

$(ROOT).pdf: *.tex
	pdflatex $(ROOT).tex
	#latex -interaction=nonstopmode $(ROOT).tex
	#dvips -o $(ROOT).ps $(ROOT).dvi
	#ps2pdf $(ROOT).ps

clean:
	rm *.dvi *.aux *.ps *.toc *.out *.log
